// The flight planner: draw a route on a map, then fly it.
//
// Two jobs in one window, deliberately:
//
//   1. SEARCH PLANNING. Give it a datum - eventually from the drift
//      model, for now typed in or taken from the drone - and it lays out
//      a proper search pattern over the water around it.
//   2. MANUAL WAYPOINTS. Click the map to drop pins and drag them about.
//      This is how the whole thing gets tested without waiting for the
//      model, and it is also what you want when a human can see where to
//      look.
//
// Both produce the same Waypoint[], reviewed by the same rules and flown
// by the same sequencer. There is no "test mode" that behaves differently
// from the real thing - the only difference is where the points came from.
//
// The map is driven imperatively (see DroneMap.tsx for the same reasoning):
// markers move continuously, and rebuilding a React tree to drag a pin
// would be the wrong tool.
import { useCallback, useEffect, useMemo, useRef, useState, useSyncExternalStore } from "react";
import L from "leaflet";
import "leaflet/dist/leaflet.css";
import "./FlightPlanner.css";
import { useDroneTelemetry } from "../services/droneTelemetry";
import {
  LIMITS,
  type LatLon,
  type SearchPattern,
  type Waypoint,
  distanceM,
  exportPlan,
  importPlan,
  numberWaypoints,
  planSearch,
  reviewPlan,
  suggestedSpacingM,
  waypoint,
  clampAlt,
} from "../services/flightPlan";
import { routeRunner } from "../services/routeRunner";

const PIN_COLOURS = {
  normal: "#FF6B35",
  active: "#35d07f",
  done: "#6b7787",
};

function pinIcon(label: string, kind: keyof typeof PIN_COLOURS): L.DivIcon {
  return L.divIcon({
    className: "wp-pin",
    iconSize: [26, 26],
    iconAnchor: [13, 13],
    html: `<div class="wp-pin-inner" style="--pin:${PIN_COLOURS[kind]}">${label}</div>`,
  });
}

const DRONE_ICON = L.divIcon({
  className: "wp-drone",
  iconSize: [28, 28],
  iconAnchor: [14, 14],
  html: `<div class="wp-drone-inner"><svg viewBox="0 0 24 24" width="28" height="28">
    <circle cx="12" cy="12" r="10" class="wp-drone-halo" />
    <path d="M12 4 L17.5 19 L12 15.6 L6.5 19 Z" class="wp-drone-body" />
  </svg></div>`,
});

function fmtDistance(m: number): string {
  return m < 1000 ? `${m.toFixed(0)} m` : `${(m / 1000).toFixed(2)} km`;
}

function fmtDuration(s: number): string {
  const mins = Math.floor(s / 60);
  const secs = Math.round(s % 60);
  return mins ? `${mins}m ${secs}s` : `${secs}s`;
}

export default function FlightPlanner() {
  const { telemetry, linkState } = useDroneTelemetry();
  const run = useSyncExternalStore(routeRunner.subscribe, routeRunner.getSnapshot);

  const gps = telemetry?.gps;
  const dronePos: LatLon | null =
    gps?.has_fix && gps.lat != null && gps.lon != null ? { lat: gps.lat, lon: gps.lon } : null;

  const [waypoints, setWaypoints] = useState<Waypoint[]>([]);
  const [altM, setAltM] = useState(6);
  const [pattern, setPattern] = useState<SearchPattern>("expanding-square");
  const [radiusM, setRadiusM] = useState(60);
  const [headingDeg, setHeadingDeg] = useState(0);
  const [spacingM, setSpacingM] = useState(() => suggestedSpacingM(6));
  const [spacingAuto, setSpacingAuto] = useState(true);
  const [datumText, setDatumText] = useState({ lat: "", lon: "" });
  const [note, setNote] = useState("");
  const [confirming, setConfirming] = useState(false);

  const containerRef = useRef<HTMLDivElement | null>(null);
  const mapRef = useRef<L.Map | null>(null);
  const pinLayerRef = useRef<L.LayerGroup | null>(null);
  const routeLineRef = useRef<L.Polyline | null>(null);
  const droneMarkerRef = useRef<L.Marker | null>(null);
  const rangeRef = useRef<L.Circle | null>(null);
  const datumRef = useRef<L.Circle | null>(null);

  // The map handler is registered once, so it must not close over stale
  // state - these refs are what it reads instead.
  const altRef = useRef(altM);
  altRef.current = altM;
  const runningRef = useRef(run.state === "flying");
  runningRef.current = run.state === "flying";

  const datum: LatLon | null = useMemo(() => {
    const lat = Number(datumText.lat);
    const lon = Number(datumText.lon);
    if (!datumText.lat || !datumText.lon) return null;
    if (!Number.isFinite(lat) || Math.abs(lat) > 90) return null;
    if (!Number.isFinite(lon) || Math.abs(lon) > 180) return null;
    return { lat, lon };
  }, [datumText]);

  const review = useMemo(() => reviewPlan(waypoints, dronePos), [waypoints, dronePos]);

  // -- map, once ---------------------------------------------------------
  useEffect(() => {
    if (!containerRef.current || mapRef.current) return;
    const map = L.map(containerRef.current, {
      center: [-25.7479, 28.2293],
      zoom: 17,
      zoomControl: true,
    });
    L.tileLayer("https://tile.openstreetmap.org/{z}/{x}/{y}.png", {
      maxZoom: 19,
      attribution: "© OpenStreetMap contributors",
    }).addTo(map);

    pinLayerRef.current = L.layerGroup().addTo(map);
    routeLineRef.current = L.polyline([], {
      color: "#FF6B35",
      weight: 2,
      opacity: 0.85,
      dashArray: "6 6",
    }).addTo(map);

    // Clicking empty water drops a pin. Refused mid-flight: editing the
    // route the aircraft is currently flying would be ambiguous about
    // which version it is following.
    map.on("click", (e: L.LeafletMouseEvent) => {
      if (runningRef.current) {
        setNote("The route is being flown. Stop it before changing the plan.");
        return;
      }
      setWaypoints((prev) =>
        numberWaypoints([...prev, waypoint(e.latlng.lat, e.latlng.lng, altRef.current)]),
      );
    });

    mapRef.current = map;
    const t = window.setTimeout(() => map.invalidateSize(), 0);
    return () => {
      window.clearTimeout(t);
      for (const ref of [routeLineRef, droneMarkerRef, rangeRef, datumRef, pinLayerRef]) {
        try {
          (ref.current as { remove?: () => void } | null)?.remove?.();
        } catch {
          /* already detached with the map */
        }
        ref.current = null;
      }
      map.remove();
      mapRef.current = null;
    };
  }, []);

  // -- pins and route line ----------------------------------------------
  useEffect(() => {
    const map = mapRef.current;
    const layer = pinLayerRef.current;
    if (!map || !layer) return;
    layer.clearLayers();

    waypoints.forEach((w, i) => {
      const kind =
        run.state === "flying" && i === run.index
          ? "active"
          : run.state !== "idle" && i < run.index
            ? "done"
            : "normal";
      const marker = L.marker([w.lat, w.lon], {
        icon: pinIcon(w.label ?? String(i + 1), kind),
        draggable: run.state !== "flying",
        zIndexOffset: 500,
      });
      marker.on("dragend", () => {
        const p = marker.getLatLng();
        setWaypoints((prev) =>
          prev.map((x) => (x.id === w.id ? { ...x, lat: p.lat, lon: p.lng } : x)),
        );
      });
      // Right-click removes, which is the fastest way to fix a misdrop.
      marker.on("contextmenu", () => {
        if (runningRef.current) return;
        setWaypoints((prev) => numberWaypoints(prev.filter((x) => x.id !== w.id)));
      });
      marker.bindTooltip(
        `${w.label ?? i + 1} · ${w.altM} m<br>${w.lat.toFixed(6)}, ${w.lon.toFixed(6)}`,
        { direction: "top", offset: [0, -12] },
      );
      marker.addTo(layer);
    });

    const line: L.LatLngExpression[] = waypoints.map((w) => [w.lat, w.lon]);
    if (dronePos && line.length) line.unshift([dronePos.lat, dronePos.lon]);
    routeLineRef.current?.setLatLngs(line);
  }, [waypoints, run.state, run.index, dronePos]);

  // -- the drone, its range ring, and the datum --------------------------
  useEffect(() => {
    const map = mapRef.current;
    if (!map) return;

    if (dronePos) {
      const ll = L.latLng(dronePos.lat, dronePos.lon);
      if (!droneMarkerRef.current) {
        droneMarkerRef.current = L.marker(ll, { icon: DRONE_ICON, interactive: false }).addTo(map);
        map.setView(ll, 18);
      } else {
        droneMarkerRef.current.setLatLng(ll);
      }
      // What the aircraft will actually accept, drawn rather than
      // described - a 300 m circle on the map is worth more than a
      // number in a panel when you are placing pins.
      if (!rangeRef.current) {
        rangeRef.current = L.circle(ll, {
          radius: LIMITS.maxRangeM,
          color: "#e9c400",
          weight: 1,
          opacity: 0.5,
          fill: false,
          dashArray: "4 8",
        }).addTo(map);
      } else {
        rangeRef.current.setLatLng(ll);
      }
    }

    if (datum) {
      const ll = L.latLng(datum.lat, datum.lon);
      if (!datumRef.current) {
        datumRef.current = L.circle(ll, {
          radius: radiusM,
          color: "#b9c7e4",
          weight: 1,
          fillColor: "#b9c7e4",
          fillOpacity: 0.08,
        }).addTo(map);
      } else {
        datumRef.current.setLatLng(ll).setRadius(radiusM);
      }
    } else if (datumRef.current) {
      datumRef.current.remove();
      datumRef.current = null;
    }
  }, [dronePos, datum, radiusM]);

  // Spacing follows height unless it has been set by hand: the stripe the
  // camera sees is a function of how high you are, so the two are not
  // independent choices.
  useEffect(() => {
    if (spacingAuto) setSpacingM(suggestedSpacingM(altM));
  }, [altM, spacingAuto]);

  const setDatumFromDrone = useCallback(() => {
    if (!dronePos) {
      setNote("No GPS fix, so there is no drone position to use.");
      return;
    }
    setDatumText({ lat: dronePos.lat.toFixed(6), lon: dronePos.lon.toFixed(6) });
  }, [dronePos]);

  const generate = useCallback(() => {
    if (!datum) {
      setNote("Set a datum first - type a latitude and longitude, or use the drone's position.");
      return;
    }
    const wps = planSearch({ datum, radiusM, spacingM, altM, pattern, headingDeg });
    setWaypoints(wps);
    setNote(`${pattern.replace("-", " ")} generated: ${wps.length} waypoints.`);
    const map = mapRef.current;
    if (map && wps.length) {
      map.fitBounds(L.latLngBounds(wps.map((w) => [w.lat, w.lon])), { padding: [40, 40] });
    }
  }, [datum, radiusM, spacingM, altM, pattern, headingDeg]);

  const fly = useCallback(async () => {
    if (review.errors.length) return;
    // Per-leg time budgets, so the runner can tell "slow" from "stuck".
    const budgets: number[] = [];
    let prev: LatLon | null = dronePos;
    for (const w of waypoints) {
      budgets.push(prev ? distanceM(prev, w) / LIMITS.cruiseMs : 60);
      prev = w;
    }
    setConfirming(false);
    setNote("");
    await routeRunner.start(waypoints, budgets);
  }, [review.errors.length, waypoints, dronePos]);

  const doExport = () => {
    const blob = new Blob(
      [exportPlan(waypoints, datum ? { datum, radiusM, spacingM, altM, pattern, headingDeg } : null)],
      { type: "application/json" },
    );
    const a = document.createElement("a");
    a.href = URL.createObjectURL(blob);
    a.download = `seayou-plan-${new Date().toISOString().slice(0, 19).replace(/[:T]/g, "")}.json`;
    a.click();
    URL.revokeObjectURL(a.href);
  };

  const doImport = (file: File) => {
    const reader = new FileReader();
    reader.onload = () => {
      const wps = importPlan(String(reader.result ?? ""));
      if (!wps) {
        setNote("That file is not a usable plan - it was rejected rather than partly loaded.");
        return;
      }
      setWaypoints(wps);
      setNote(`Loaded ${wps.length} waypoints.`);
      const map = mapRef.current;
      if (map) map.fitBounds(L.latLngBounds(wps.map((w) => [w.lat, w.lon])), { padding: [40, 40] });
    };
    reader.readAsText(file);
  };

  const flying = run.state === "flying";
  const canFly = waypoints.length > 0 && review.errors.length === 0 && linkState === "connected";

  // The aircraft's own return reserve. Shown here because this is the
  // screen where someone decides to send it 250 m away, and the honest
  // answer to "can it get back?" lives on the aircraft, not in the plan.
  const guard = telemetry?.battery_guard;
  const reserve = guard?.reserve ?? null;
  const headroom = reserve?.headroom_v ?? null;
  const reserveState =
    guard && (guard.level === "return" || guard.level === "land_now")
      ? "spent"
      : headroom !== null && headroom < 0.05
        ? "low"
        : "ok";

  return (
    <div className="planner">
      <div ref={containerRef} className="planner-map" />

      <aside className="planner-panel">
        <header className="planner-head">
          <h2>FLIGHT PLANNER</h2>
          <p>
            Click the map to drop a waypoint. Drag to move it, right-click to remove it.
          </p>
        </header>

        {/* Search pattern ------------------------------------------------ */}
        <section className="planner-block">
          <h3>SEARCH PATTERN</h3>
          <p className="planner-hint">
            The drift model will put its predicted position here. Until it exists, type a
            datum or take the drone&apos;s own.
          </p>
          <div className="planner-row">
            <label>
              <span>DATUM LAT</span>
              <input
                value={datumText.lat}
                onChange={(e) => setDatumText((d) => ({ ...d, lat: e.target.value }))}
                placeholder="-34.108300"
                inputMode="decimal"
              />
            </label>
            <label>
              <span>DATUM LON</span>
              <input
                value={datumText.lon}
                onChange={(e) => setDatumText((d) => ({ ...d, lon: e.target.value }))}
                placeholder="18.470000"
                inputMode="decimal"
              />
            </label>
          </div>
          <button className="planner-btn ghost" onClick={setDatumFromDrone} disabled={!dronePos}>
            USE THE DRONE&apos;S POSITION
          </button>

          <div className="planner-row">
            <label>
              <span>PATTERN</span>
              <select value={pattern} onChange={(e) => setPattern(e.target.value as SearchPattern)}>
                <option value="expanding-square">Expanding square</option>
                <option value="parallel-track">Parallel track</option>
                <option value="sector">Sector search</option>
              </select>
            </label>
            <label>
              <span>RADIUS m</span>
              <input
                type="number"
                min={5}
                max={LIMITS.maxRangeM}
                value={radiusM}
                onChange={(e) => setRadiusM(Math.max(5, Number(e.target.value) || 0))}
              />
            </label>
          </div>
          <div className="planner-row">
            <label>
              <span>HEIGHT m</span>
              <input
                type="number"
                min={LIMITS.minAltM}
                max={LIMITS.maxAltM}
                value={altM}
                onChange={(e) => setAltM(clampAlt(Number(e.target.value) || LIMITS.minAltM))}
              />
            </label>
            <label>
              <span>
                SPACING m{" "}
                <button
                  className="planner-link"
                  onClick={() => setSpacingAuto((v) => !v)}
                  title="Derive the spacing from the camera's ground swath at this height"
                >
                  {spacingAuto ? "auto" : "manual"}
                </button>
              </span>
              <input
                type="number"
                min={2}
                value={spacingM}
                disabled={spacingAuto}
                onChange={(e) => setSpacingM(Math.max(2, Number(e.target.value) || 2))}
              />
            </label>
          </div>
          <label className="planner-full">
            <span>SWEEP HEADING {headingDeg}&deg;</span>
            <input
              type="range"
              min={0}
              max={350}
              step={10}
              value={headingDeg}
              onChange={(e) => setHeadingDeg(Number(e.target.value))}
            />
          </label>
          <button className="planner-btn" onClick={generate} disabled={flying}>
            GENERATE SEARCH
          </button>
        </section>

        {/* Plan ---------------------------------------------------------- */}
        <section className="planner-block">
          <h3>
            PLAN <span className="planner-count">{waypoints.length}</span>
          </h3>
          {waypoints.length > 0 && (
            <div className="planner-stats">
              <div>
                <em>DISTANCE</em>
                {fmtDistance(review.totalM)}
              </div>
              <div>
                <em>FLIGHT TIME</em>
                {fmtDuration(review.etaS)}
              </div>
              <div>
                <em>LONGEST LEG</em>
                {fmtDistance(review.longestLegM)}
              </div>
            </div>
          )}

          <ol className="planner-list">
            {waypoints.map((w, i) => (
              <li
                key={w.id}
                className={
                  flying && i === run.index ? "is-active" : run.state !== "idle" && i < run.index ? "is-done" : ""
                }
              >
                <span className="wp-no">{w.label ?? i + 1}</span>
                <span className="wp-ll">
                  {w.lat.toFixed(5)}, {w.lon.toFixed(5)}
                </span>
                <input
                  className="wp-alt"
                  type="number"
                  min={LIMITS.minAltM}
                  max={LIMITS.maxAltM}
                  value={w.altM}
                  disabled={flying}
                  onChange={(e) =>
                    setWaypoints((prev) =>
                      prev.map((x) =>
                        x.id === w.id ? { ...x, altM: clampAlt(Number(e.target.value) || LIMITS.minAltM) } : x,
                      ),
                    )
                  }
                />
                <button
                  className="wp-del"
                  disabled={flying}
                  onClick={() => setWaypoints((prev) => numberWaypoints(prev.filter((x) => x.id !== w.id)))}
                  title="Remove this waypoint"
                >
                  &times;
                </button>
              </li>
            ))}
          </ol>

          {waypoints.length === 0 && (
            <p className="planner-empty">
              No waypoints yet. Click the map, or generate a search pattern above.
            </p>
          )}

          <div className="planner-row planner-tools">
            <button className="planner-btn ghost" onClick={() => setWaypoints([])} disabled={flying || !waypoints.length}>
              CLEAR
            </button>
            <button className="planner-btn ghost" onClick={doExport} disabled={!waypoints.length}>
              EXPORT
            </button>
            <label className="planner-btn ghost as-label">
              IMPORT
              <input
                type="file"
                accept="application/json,.json"
                onChange={(e) => {
                  const f = e.target.files?.[0];
                  if (f) doImport(f);
                  e.target.value = "";
                }}
              />
            </label>
          </div>
        </section>

        {/* Problems ------------------------------------------------------ */}
        {(review.errors.length > 0 || review.warnings.length > 0) && (
          <section className="planner-block">
            {review.errors.map((e) => (
              <p key={e} className="planner-error">
                {e}
              </p>
            ))}
            {review.warnings.map((w) => (
              <p key={w} className="planner-warn">
                {w}
              </p>
            ))}
          </section>
        )}

        {/* Getting home -------------------------------------------------- */}
        {guard && (
          <section className="planner-block">
            <h3>RETURN RESERVE</h3>
            <div className={`planner-reserve is-${reserveState}`}>
              {reserve && reserve.distance_home_m !== null ? (
                <>
                  <div className="res-row">
                    <em>HOME</em>
                    {reserve.distance_home_m.toFixed(0)} m away
                    {reserve.time_home_s !== null && ` · ${Math.round(reserve.time_home_s)} s to fly`}
                  </div>
                  <div className="res-row">
                    <em>TURNS BACK AT</em>
                    {reserve.return_at_v.toFixed(2)} V/cell
                    {` (${guard.thresholds.return.toFixed(2)} + ${reserve.reserve_v.toFixed(2)} reserve)`}
                  </div>
                  <div className="res-row">
                    <em>HEADROOM</em>
                    {headroom !== null ? `${headroom.toFixed(2)} V/cell` : "--"}
                    {guard.cell_v !== null && ` · now ${guard.cell_v.toFixed(2)}`}
                  </div>
                  <p className="res-note">
                    {reserveState === "spent"
                      ? "The aircraft has taken over and is coming back. The route is cancelled."
                      : reserveState === "low"
                        ? "Close to turning back on its own. Expect the search to be cut short."
                        : reserve.measured
                          ? `Reserve is sized from the measured discharge, ${reserve.fall_v_per_min?.toFixed(3)} V/cell per minute. It grows as the drone goes further out.`
                          : "Reserve is using a default discharge rate until it has measured this pack in flight."}
                  </p>
                </>
              ) : (
                <p className="res-note">
                  No fix or no home recorded, so the distance-aware reserve is not
                  active. The aircraft falls back to its fixed{" "}
                  {guard.thresholds.return.toFixed(2)} V/cell return threshold.
                </p>
              )}
            </div>
          </section>
        )}

        {/* Flying -------------------------------------------------------- */}
        <section className="planner-block planner-fly">
          <h3>FLY THE PLAN</h3>
          <p className="planner-hint">
            This sends positions only. The aircraft must already be airborne and holding -
            take off from the flight controls on Live Feeds first.
          </p>

          <div className={`planner-run state-${run.state}${run.problem ? " is-problem" : ""}`}>
            <strong>
              {run.state === "flying"
                ? `LEG ${run.index + 1} / ${run.total}`
                : run.state.toUpperCase()}
            </strong>
            <span>{run.message}</span>
          </div>

          {!flying && !confirming && (
            <button className="planner-btn danger" onClick={() => setConfirming(true)} disabled={!canFly}>
              FLY PLAN
            </button>
          )}

          {!flying && confirming && (
            <div className="planner-confirm">
              <p>
                This will fly the aircraft through {waypoints.length} waypoints, about{" "}
                {fmtDistance(review.totalM)} and {fmtDuration(review.etaS)}. Autonomous flight
                on this airframe is not proven - keep a hand on the sticks, and remember that
                moving them stops the route.
              </p>
              <div className="planner-row">
                <button className="planner-btn danger" onClick={fly}>
                  YES, FLY IT
                </button>
                <button className="planner-btn ghost" onClick={() => setConfirming(false)}>
                  CANCEL
                </button>
              </div>
            </div>
          )}

          {flying && (
            <button className="planner-btn danger" onClick={() => routeRunner.abort()}>
              STOP ROUTE
            </button>
          )}

          {run.state === "stopped" && run.index >= 0 && run.index + 1 < run.total && (
            <button className="planner-btn ghost" onClick={() => routeRunner.skip()}>
              SKIP TO LEG {run.index + 2}
            </button>
          )}

          {linkState !== "connected" && (
            <p className="planner-error">No link to the drone, so nothing can be flown.</p>
          )}
        </section>

        {note && <p className="planner-note">{note}</p>}
      </aside>
    </div>
  );
}
