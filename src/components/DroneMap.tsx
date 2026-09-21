// Live map of where the drone is and how it is moving.
//
// Reads the same telemetry stream as everything else (droneLink -> the Pi's
// server.py -> gps.py -> the u-blox USB dongle) and is strictly read-only:
// it never takes the control slot, so having the map open costs nothing and
// cannot interfere with whoever is flying.
//
// Three rules this component follows, all for the same reason - a map is
// believed far more readily than a number, so a map that lies is dangerous:
//
//   1. No fix, no marker. It shows an "acquiring" state instead of putting
//      the aircraft at 0,0 or at its last known position.
//   2. The trail is only ever drawn from real fixes. Nothing is
//      interpolated, and a gap in the fixes stays a gap.
//   3. The "stale" badge appears the moment fixes stop arriving, because a
//      marker frozen on screen otherwise looks exactly like a drone hovering
//      perfectly still.
//
// Leaflet is driven imperatively through a ref rather than with
// react-leaflet: this needs one map, updated 30 times a second, and
// rebuilding a React tree at that rate to move one marker would be silly.
import { useEffect, useMemo, useRef, useState } from "react";
import L from "leaflet";
import "leaflet/dist/leaflet.css";
import "./DroneMap.css";
import { useDroneTelemetry } from "../services/droneTelemetry";
import type { DroneGps } from "../services/droneTelemetry";

/** A fix worth keeping in the trail. */
interface TrackPoint {
  lat: number;
  lon: number;
  t: number;
}

/** Ignore sub-metre jitter - a hovering drone would otherwise draw a blob. */
const MIN_TRAIL_STEP_M = 1.0;

/** Roughly an hour of flying at 1 point/metre. Oldest are dropped first. */
const MAX_TRAIL_POINTS = 3000;

const EARTH_R_M = 6371000;

function distanceM(a: TrackPoint, b: TrackPoint): number {
  const p1 = (a.lat * Math.PI) / 180;
  const p2 = (b.lat * Math.PI) / 180;
  const dp = p2 - p1;
  const dl = ((b.lon - a.lon) * Math.PI) / 180;
  const h =
    Math.sin(dp / 2) ** 2 +
    Math.cos(p1) * Math.cos(p2) * Math.sin(dl / 2) ** 2;
  return 2 * EARTH_R_M * Math.asin(Math.min(1, Math.sqrt(h)));
}

/** Bearing a -> b in degrees true, for when the GPS withholds course. */
function bearingDeg(a: TrackPoint, b: TrackPoint): number {
  const p1 = (a.lat * Math.PI) / 180;
  const p2 = (b.lat * Math.PI) / 180;
  const dl = ((b.lon - a.lon) * Math.PI) / 180;
  const y = Math.sin(dl) * Math.cos(p2);
  const x =
    Math.cos(p1) * Math.sin(p2) - Math.sin(p1) * Math.cos(p2) * Math.cos(dl);
  return ((Math.atan2(y, x) * 180) / Math.PI + 360) % 360;
}

function fmtCoord(v: number | null | undefined, isLat: boolean): string {
  if (v === null || v === undefined) return "--";
  const hemi = isLat ? (v >= 0 ? "N" : "S") : v >= 0 ? "E" : "W";
  return `${Math.abs(v).toFixed(6)}° ${hemi}`;
}

function fmtDistance(m: number): string {
  return m < 1000 ? `${m.toFixed(0)} m` : `${(m / 1000).toFixed(2)} km`;
}

/**
 * The arrow that IS the drone. A div icon rather than an image so it can be
 * rotated to the heading with plain CSS and needs no marker-icon asset -
 * Leaflet's default PNG icons break under Vite's bundler.
 */
function droneIcon(headingDeg: number, stale: boolean): L.DivIcon {
  return L.divIcon({
    className: "drone-marker",
    iconSize: [34, 34],
    iconAnchor: [17, 17],
    html: `
      <div class="drone-marker-inner${stale ? " is-stale" : ""}"
           style="transform: rotate(${headingDeg}deg)">
        <svg viewBox="0 0 24 24" width="34" height="34" aria-hidden="true">
          <circle cx="12" cy="12" r="11" class="drone-marker-halo" />
          <path d="M12 3 L18.5 20 L12 16.2 L5.5 20 Z" class="drone-marker-body" />
        </svg>
      </div>`,
  });
}

const HOME_ICON = L.divIcon({
  className: "home-marker",
  iconSize: [14, 14],
  iconAnchor: [7, 7],
  html: `<div class="home-marker-inner"></div>`,
});

/** Pixels of the map hidden behind whatever the page floats over it. */
export interface MapInsets {
  top: number;
  right: number;
  bottom: number;
  left: number;
}

const NO_INSETS: MapInsets = { top: 0, right: 0, bottom: 0, left: 0 };

interface DroneMapProps {
  /** Fills its parent - give that parent a height. */
  className?: string;
  /**
   * Edges of the map that are covered by the host page's own floating
   * panels. Centring and "fit track" both work to the VISIBLE rectangle
   * instead of the whole element, so the drone never ends up neatly
   * centred underneath a panel - which looks exactly like a drone that
   * has vanished. Ignored below the md breakpoint, where those panels
   * are not rendered.
   */
  insets?: MapInsets;
}

export default function DroneMap({
  className = "",
  insets = NO_INSETS,
}: DroneMapProps) {
  const { telemetry, linkState, packetAgeSeconds } = useDroneTelemetry();
  const gps: DroneGps | undefined = telemetry?.gps;
  const hasFix = Boolean(gps?.has_fix && gps?.lat != null && gps?.lon != null);

  const containerRef = useRef<HTMLDivElement | null>(null);
  const mapRef = useRef<L.Map | null>(null);
  const droneMarkerRef = useRef<L.Marker | null>(null);
  const homeMarkerRef = useRef<L.Marker | null>(null);
  const trailRef = useRef<L.Polyline | null>(null);
  const accuracyRef = useRef<L.Circle | null>(null);
  const trackRef = useRef<TrackPoint[]>([]);

  // Follow is opt-out: the map keeps the drone centred until you drag it
  // away, and a button brings it back. Being yanked back to centre while
  // trying to look at something is the most annoying thing a tracking map
  // can do.
  const [follow, setFollow] = useState(true);
  const followRef = useRef(follow);
  followRef.current = follow;

  const [tilesFailed, setTilesFailed] = useState(false);
  const [trailVersion, setTrailVersion] = useState(0); // redraws the readout

  // Read through a ref: the panning effect below must not re-subscribe
  // every time the parent re-renders with a fresh insets object.
  const insetsRef = useRef(insets);
  insetsRef.current = insets;

  /**
   * Where a latlng has to be placed so it lands in the middle of the part
   * of the map you can actually SEE. With no insets this is just the
   * point itself.
   */
  function centreFor(map: L.Map, latlng: L.LatLng): L.LatLng {
    const i = window.innerWidth < 768 ? NO_INSETS : insetsRef.current;
    if (!i.top && !i.right && !i.bottom && !i.left) return latlng;
    const size = map.getSize();
    const shiftX = (i.left - i.right) / 2;
    const shiftY = (i.top - i.bottom) / 2;
    if (Math.abs(shiftX) > size.x / 3 || Math.abs(shiftY) > size.y / 3) {
      // The panels cover so much that recentring would push the drone off
      // the map entirely. Better plainly centred than cleverly hidden.
      return latlng;
    }
    return map.containerPointToLatLng(
      map.latLngToContainerPoint(latlng).subtract([shiftX, shiftY]),
    );
  }

  // -- map construction, once ------------------------------------------
  useEffect(() => {
    if (!containerRef.current || mapRef.current) return;

    const map = L.map(containerRef.current, {
      center: [-25.7479, 28.2293], // Pretoria - a placeholder until the
      zoom: 17,                    // first fix, never shown as a position
      zoomControl: true,
      attributionControl: true,
      // NOT preferCanvas. The canvas renderer keeps a scheduled redraw
      // alive past map.remove(), and it then throws on every frame
      // ("cannot read properties of undefined (reading 'clearRect')")
      // the moment you navigate off this page. One polyline and two
      // markers do not need canvas anyway.
    });

    const tiles = L.tileLayer(
      "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
      {
        maxZoom: 19,
        attribution: "© OpenStreetMap contributors",
        crossOrigin: true,
      },
    );
    // No internet at the flying field is the normal case, not an error.
    // The basemap simply does not appear and the trail draws on the dark
    // background instead, which is still a usable picture of the flight.
    tiles.on("tileerror", () => setTilesFailed(true));
    tiles.on("tileload", () => setTilesFailed(false));
    tiles.addTo(map);

    trailRef.current = L.polyline([], {
      color: "#FF6B35",
      weight: 3,
      opacity: 0.9,
      lineJoin: "round",
    }).addTo(map);

    accuracyRef.current = L.circle([0, 0], {
      radius: 0,
      color: "#FF6B35",
      weight: 1,
      opacity: 0.35,
      fillColor: "#FF6B35",
      fillOpacity: 0.08,
    });

    // Any manual pan is a decision to stop following.
    map.on("dragstart", () => setFollow(false));

    mapRef.current = map;
    // Leaflet measures its container on creation; inside a flex/absolute
    // layout that size is often still zero at that moment.
    const t = window.setTimeout(() => map.invalidateSize(), 0);

    return () => {
      window.clearTimeout(t);
      // Tear the layers down explicitly before the map goes. Leaving
      // them attached to a removed map is what turns navigating away
      // from this page into a stream of console errors, and a layer
      // holding a dead renderer is a leak whether or not it throws.
      for (const ref of [trailRef, droneMarkerRef, homeMarkerRef, accuracyRef]) {
        try {
          ref.current?.remove();
        } catch {
          /* already detached with the map - nothing to do */
        }
        ref.current = null;
      }
      map.remove();
      mapRef.current = null;
    };
  }, []);

  // -- every fix ---------------------------------------------------------
  useEffect(() => {
    const map = mapRef.current;
    if (!map || !hasFix || gps?.lat == null || gps?.lon == null) return;

    const point: TrackPoint = { lat: gps.lat, lon: gps.lon, t: Date.now() };
    const track = trackRef.current;
    const last = track[track.length - 1];

    if (!last || distanceM(last, point) >= MIN_TRAIL_STEP_M) {
      track.push(point);
      if (track.length > MAX_TRAIL_POINTS) track.shift();
      trailRef.current?.setLatLngs(track.map((p) => [p.lat, p.lon]));
      setTrailVersion((v) => v + 1);

      // The first fix of the session is home - the same point the
      // battery failsafe would return to.
      if (track.length === 1) {
        homeMarkerRef.current = L.marker([point.lat, point.lon], {
          icon: HOME_ICON,
          title: "Home - first fix of this session",
          interactive: false,
        }).addTo(map);
      }
    }

    // Heading: the GPS's own course when it is moving fast enough to have
    // one, the aircraft's compass yaw otherwise. Course over ground is the
    // truthful one for a trail; yaw is which way the nose points, which is
    // all there is while hovering.
    const heading =
      gps.course_deg ??
      (last && distanceM(last, point) >= MIN_TRAIL_STEP_M
        ? bearingDeg(last, point)
        : (telemetry?.attitude?.yaw ?? 0));

    const stale = (packetAgeSeconds ?? 0) > 3;
    const latlng = L.latLng(point.lat, point.lon);

    if (!droneMarkerRef.current) {
      droneMarkerRef.current = L.marker(latlng, {
        icon: droneIcon(heading, stale),
        zIndexOffset: 1000,
        interactive: false,
      }).addTo(map);
      map.setView(centreFor(map, latlng), 18);
    } else {
      droneMarkerRef.current.setLatLng(latlng);
      droneMarkerRef.current.setIcon(droneIcon(heading, stale));
    }

    // HDOP is not metres, but hdop * ~5 m is the usual rule of thumb for
    // a consumer receiver and gives an honest sense of "somewhere in
    // here" rather than the false precision of a single pixel.
    const circle = accuracyRef.current;
    if (circle) {
      if (gps.hdop != null && gps.hdop < 50) {
        circle.setLatLng(latlng).setRadius(Math.max(2, gps.hdop * 5));
        if (!map.hasLayer(circle)) circle.addTo(map);
      } else if (map.hasLayer(circle)) {
        circle.remove();
      }
    }

    if (followRef.current) {
      map.panTo(centreFor(map, latlng), { animate: true, duration: 0.25 });
    }
  }, [gps, hasFix, telemetry?.attitude?.yaw, packetAgeSeconds]);

  // -- derived readouts --------------------------------------------------
  const track = trackRef.current;
  const stats = useMemo(() => {
    void trailVersion; // recompute whenever the trail grows
    const pts = trackRef.current;
    if (pts.length === 0) return { home: 0, travelled: 0, points: 0 };
    const home = pts[0];
    const now = pts[pts.length - 1];
    let travelled = 0;
    for (let i = 1; i < pts.length; i++) travelled += distanceM(pts[i - 1], pts[i]);
    return { home: distanceM(home, now), travelled, points: pts.length };
  }, [trailVersion]);

  const stale = (packetAgeSeconds ?? 0) > 3;

  function clearTrail() {
    trackRef.current = [];
    trailRef.current?.setLatLngs([]);
    homeMarkerRef.current?.remove();
    homeMarkerRef.current = null;
    setTrailVersion((v) => v + 1);
  }

  function recentre() {
    setFollow(true);
    const pts = trackRef.current;
    const map = mapRef.current;
    if (map && pts.length) {
      const p = pts[pts.length - 1];
      map.setView(centreFor(map, L.latLng(p.lat, p.lon)),
                  Math.max(map.getZoom(), 17));
    }
  }

  function fitTrack() {
    const map = mapRef.current;
    const pts = trackRef.current;
    if (!map || pts.length < 2) return;
    setFollow(false);
    const i = window.innerWidth < 768 ? NO_INSETS : insetsRef.current;
    map.fitBounds(L.latLngBounds(pts.map((p) => [p.lat, p.lon])), {
      paddingTopLeft: [i.left + 24, i.top + 24],
      paddingBottomRight: [i.right + 24, i.bottom + 24],
    });
  }

  return (
    <div className={`drone-map-wrap ${className}`}>
      <div ref={containerRef} className="drone-map-canvas" />

      {/* Status strip - the honest state of the fix, always visible. */}
      <div className="drone-map-status">
        <span
          className={`dm-dot ${
            linkState !== "connected"
              ? "dm-dot-off"
              : !hasFix
                ? "dm-dot-wait"
                : stale
                  ? "dm-dot-stale"
                  : "dm-dot-ok"
          }`}
        />
        <span className="dm-state">
          {linkState !== "connected"
            ? "DRONE OFFLINE"
            : !hasFix
              ? `NO FIX · ${gps?.sats_in_view ?? 0} SATS IN VIEW`
              : stale
                ? "FIX STALE"
                : `FIX · ${gps?.sat_count ?? 0} SATS`}
        </span>
        {hasFix && (
          <>
            <span className="dm-sep" />
            <span className="dm-field">
              <em>LAT</em> {fmtCoord(gps?.lat, true)}
            </span>
            <span className="dm-field">
              <em>LON</em> {fmtCoord(gps?.lon, false)}
            </span>
            <span className="dm-field">
              <em>SPD</em>{" "}
              {gps?.speed_ms != null ? `${gps.speed_ms.toFixed(1)} m/s` : "--"}
            </span>
            <span className="dm-field">
              <em>ALT</em>{" "}
              {gps?.alt_m != null ? `${gps.alt_m.toFixed(0)} m` : "--"}
            </span>
            <span className="dm-field">
              <em>HOME</em> {fmtDistance(stats.home)}
            </span>
            <span className="dm-field">
              <em>TRACK</em> {fmtDistance(stats.travelled)}
            </span>
          </>
        )}
      </div>

      {/* Map controls. */}
      <div className="drone-map-buttons">
        <button
          className={`dm-btn ${follow ? "is-on" : ""}`}
          onClick={() => (follow ? setFollow(false) : recentre())}
          title="Keep the map centred on the drone"
        >
          <span className="material-symbols-outlined" data-icon="person_pin_circle">person_pin_circle</span>
          {follow ? "FOLLOWING" : "FOLLOW"}
        </button>
        <button className="dm-btn" onClick={fitTrack} disabled={track.length < 2} title="Zoom to the whole flight">
          <span className="material-symbols-outlined" data-icon="zoom_in">zoom_in</span>
          FIT TRACK
        </button>
        <button className="dm-btn" onClick={clearTrail} disabled={track.length === 0} title="Erase the trail">
          <span className="material-symbols-outlined" data-icon="remove">remove</span>
          CLEAR
        </button>
      </div>

      {/* Waiting-for-fix card. Shown instead of a marker, never as well as. */}
      {linkState === "connected" && !hasFix && (
        <div
          className="drone-map-waiting"
          // Centred on the VISIBLE map for the same reason the drone is -
          // a "waiting for a fix" card tucked behind a panel tells nobody
          // anything.
          style={{
            transform: `translate(calc(-50% + ${
              (insets.left - insets.right) / 2
            }px), calc(-50% + ${(insets.top - insets.bottom) / 2}px))`,
          }}
        >
          <span className="material-symbols-outlined dm-wait-icon" data-icon="explore">explore</span>
          <p className="dm-wait-title">Waiting for a GPS fix</p>
          <p className="dm-wait-body">
            {gps?.sats_in_view ?? 0} satellites in view
            {gps?.best_snr ? `, best ${gps.best_snr} dB-Hz` : ""}. A cold start
            needs open sky and can take several minutes. Indoors it will
            never fix at all.
          </p>
        </div>
      )}

      {tilesFailed && (
        <div className="drone-map-offline">
          NO BASEMAP &middot; OFFLINE &mdash; track still recording
        </div>
      )}
    </div>
  );
}
