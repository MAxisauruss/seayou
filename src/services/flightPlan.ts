// Flight planning: turn a point on the water into a route the drone can fly.
//
// Two ways in, one way out:
//
//   * The predictive drift model (not built yet) will hand over a datum -
//     where it thinks the casualty is - plus how sure it is. `planSearch`
//     turns that into a search pattern.
//   * A person drops pins on a map. Same Waypoint[] out the other side.
//
// Everything here is PURE: no network, no Leaflet, no React. That is what
// makes it testable without an aircraft, and it is the reason the search
// patterns can be checked against known geometry rather than eyeballed on
// a map.
//
// --------------------------------------------------------------------
// The limits below are copied from groundstation/mission.py on purpose.
// --------------------------------------------------------------------
// They are duplicated, not imported, because they live in Python on the
// other side of a socket. This copy exists so the UI can warn BEFORE a
// leg is sent; the server still enforces its own and is the authority. If
// mission.py's limits change, change these too - a planner that thinks
// the range is larger than the aircraft does just produces refusals at
// leg seven, halfway through a search.
export const LIMITS = {
  /** Largest distance from the aircraft to a target, metres. */
  maxRangeM: 300,
  minAltM: 1,
  maxAltM: 30,
  maxTakeoffAltM: 10,
  /** What the guidance aims for, m/s. Used for time estimates only. */
  cruiseMs: 3,
  /** Inside this the mission calls itself arrived. */
  arriveRadiusM: 2.5,
};

const EARTH_R_M = 6371000;
const DEG = Math.PI / 180;

export interface LatLon {
  lat: number;
  lon: number;
}

export interface Waypoint extends LatLon {
  /** Stable across reorders and edits, so React keys and the runner agree. */
  id: string;
  /** Metres above the height baseline the aircraft took at link-up. */
  altM: number;
  /** Shown on the map pin. Auto-numbered unless a pattern named it. */
  label?: string;
}

export type SearchPattern = "expanding-square" | "parallel-track" | "sector";

export interface SearchSpec {
  /** Where the casualty is predicted to be. */
  datum: LatLon;
  /** How far out to search from the datum, metres. */
  radiusM: number;
  pattern: SearchPattern;
  /** Distance between adjacent sweeps, metres. See suggestedSpacingM. */
  spacingM: number;
  altM: number;
  /**
   * Which way the sweeps run, degrees true. For a drift search this
   * should normally be set ACROSS the drift, so the aircraft crosses the
   * likely track rather than running along it.
   */
  headingDeg?: number;
}

// --------------------------------------------------------------------
// Geometry
// --------------------------------------------------------------------

/** Metres per degree of latitude. Near enough constant. */
const M_PER_DEG_LAT = (Math.PI * EARTH_R_M) / 180;

/** Metres per degree of longitude SHRINKS towards the poles. */
function mPerDegLon(lat: number): number {
  return M_PER_DEG_LAT * Math.cos(lat * DEG);
}

/**
 * Offset a point by a local east/north displacement in metres.
 *
 * A flat-earth approximation, which is correct to well under a metre at
 * the scale this aircraft flies (a 300 m box). Anything bigger would want
 * a proper geodesic, and would also be refused by the range limit.
 */
export function offsetMetres(origin: LatLon, eastM: number, northM: number): LatLon {
  return {
    lat: origin.lat + northM / M_PER_DEG_LAT,
    lon: origin.lon + eastM / mPerDegLon(origin.lat),
  };
}

/** Great-circle distance in metres. */
export function distanceM(a: LatLon, b: LatLon): number {
  const p1 = a.lat * DEG;
  const p2 = b.lat * DEG;
  const dp = p2 - p1;
  const dl = (b.lon - a.lon) * DEG;
  const h =
    Math.sin(dp / 2) ** 2 + Math.cos(p1) * Math.cos(p2) * Math.sin(dl / 2) ** 2;
  return 2 * EARTH_R_M * Math.asin(Math.min(1, Math.sqrt(h)));
}

/** Initial bearing a -> b, degrees true. */
export function bearingDeg(a: LatLon, b: LatLon): number {
  const p1 = a.lat * DEG;
  const p2 = b.lat * DEG;
  const dl = (b.lon - a.lon) * DEG;
  const y = Math.sin(dl) * Math.cos(p2);
  const x = Math.cos(p1) * Math.sin(p2) - Math.sin(p1) * Math.cos(p2) * Math.cos(dl);
  return ((Math.atan2(y, x) / DEG) + 360) % 360;
}

/** Rotate a local east/north vector by a heading, degrees true. */
function rotate(eastM: number, northM: number, headingDeg: number): [number, number] {
  const t = headingDeg * DEG;
  // Heading is clockwise from north, so north is the axis being turned.
  return [
    eastM * Math.cos(t) + northM * Math.sin(t),
    -eastM * Math.sin(t) + northM * Math.cos(t),
  ];
}

let seq = 0;
function makeId(): string {
  seq += 1;
  return `wp${seq}-${Math.random().toString(36).slice(2, 7)}`;
}

/** A waypoint with an id, for dropping a pin. */
export function waypoint(lat: number, lon: number, altM: number, label?: string): Waypoint {
  return { id: makeId(), lat, lon, altM, label };
}

// --------------------------------------------------------------------
// How wide a stripe the camera actually sees
// --------------------------------------------------------------------

/**
 * Spacing that gives the requested overlap between passes, metres.
 *
 * Ground swath for a downward camera is 2 * height * tan(fov / 2). The
 * Pi camera module 3 is about 66 degrees horizontally; at 8 m that is a
 * 10 m stripe, so sweeps 7 m apart overlap by 30 %.
 *
 * Overlap is not padding. A drifting casualty is a few pixels, the
 * aircraft yaws in wind, and the barometer wanders by a metre - sweeps
 * planned edge-to-edge will leave unsearched water between them.
 */
export function suggestedSpacingM(altM: number, fovDeg = 66, overlap = 0.3): number {
  const swath = 2 * altM * Math.tan((fovDeg / 2) * DEG);
  return Math.max(2, Math.round(swath * (1 - overlap) * 10) / 10);
}

// --------------------------------------------------------------------
// Search patterns
// --------------------------------------------------------------------

/**
 * Expanding square: start ON the datum and spiral outward in squares.
 *
 * The pattern to use when the datum is good and the casualty is likely
 * close to it - it searches the highest-probability water first, which
 * matters when the thing you are racing is how long someone stays afloat.
 */
function expandingSquare(spec: SearchSpec): Waypoint[] {
  const { datum, radiusM, spacingM, altM } = spec;
  const heading = spec.headingDeg ?? 0;
  const out: Waypoint[] = [waypoint(datum.lat, datum.lon, altM, "Datum")];

  let east = 0;
  let north = 0;
  let leg = spacingM;
  let dir = 0; // 0 N, 1 E, 2 S, 3 W - before rotation by `heading`
  // Two legs of each length, then the length grows: the square spiral.
  for (let i = 0; i < 400; i++) {
    let e2 = east;
    let n2 = north;
    switch (dir) {
      case 0: n2 += leg; break;
      case 1: e2 += leg; break;
      case 2: n2 -= leg; break;
      default: e2 -= leg;
    }
    // Stop on the OFFSET, not on the leg length. Testing against the leg
    // let the last few turns reach about 1.4x the radius - a spiral drawn
    // visibly outside the search circle on the map, searching water the
    // operator did not ask for.
    if (Math.max(Math.abs(e2), Math.abs(n2)) > radiusM) break;
    east = e2;
    north = n2;
    const [e, n] = rotate(east, north, heading);
    const p = offsetMetres(datum, e, n);
    out.push(waypoint(p.lat, p.lon, altM));
    dir = (dir + 1) % 4;
    if (i % 2 === 1) leg += spacingM;
  }
  return out;
}

/**
 * Parallel track: mow the lawn across a box centred on the datum.
 *
 * The pattern for an uncertain datum over a wide area - it covers evenly
 * rather than favouring the centre. `headingDeg` sets the direction of
 * the long legs; run them ACROSS the predicted drift so each pass cuts
 * the casualty's likely track.
 */
function parallelTrack(spec: SearchSpec): Waypoint[] {
  const { datum, radiusM, spacingM, altM } = spec;
  const heading = spec.headingDeg ?? 0;
  const out: Waypoint[] = [];
  const half = radiusM;
  // Offset the first and last sweeps inward by half a spacing, so the box
  // edges get the same coverage as the middle instead of a half-width
  // stripe of unsearched water.
  const lanes = Math.max(1, Math.ceil((2 * half) / spacingM));
  for (let i = 0; i <= lanes; i++) {
    const across = -half + (i * 2 * half) / lanes;
    // Serpentine: every other lane runs back the other way, so the
    // aircraft never flies a dead leg across the whole box.
    const ends = i % 2 === 0 ? [-half, half] : [half, -half];
    for (const along of ends) {
      const [e, n] = rotate(across, along, heading);
      const p = offsetMetres(datum, e, n);
      out.push(waypoint(p.lat, p.lon, altM));
    }
  }
  return out;
}

/**
 * Sector search: three 120-degree wedges through the datum.
 *
 * Crosses the datum repeatedly from different directions, which is what
 * you want for a small object that is easy to miss on any single pass -
 * sun glare, a wave, a wrong glance. Standard in maritime SAR for a
 * tight datum.
 */
function sector(spec: SearchSpec): Waypoint[] {
  const { datum, radiusM, altM } = spec;
  const heading = spec.headingDeg ?? 0;
  const out: Waypoint[] = [waypoint(datum.lat, datum.lon, altM, "Datum")];
  // Nine legs: three sectors of three, each turned 120 degrees.
  for (let i = 0; i < 9; i++) {
    const a = heading + i * 120 + Math.floor(i / 3) * 30;
    const e = radiusM * Math.sin(a * DEG);
    const n = radiusM * Math.cos(a * DEG);
    const p = offsetMetres(datum, e, n);
    out.push(waypoint(p.lat, p.lon, altM));
    if (i % 3 === 2) {
      out.push(waypoint(datum.lat, datum.lon, altM, "Datum"));
    }
  }
  return out;
}

/**
 * Build a search route around a predicted position.
 *
 * This is the seam the drift model plugs into: give it a datum and how
 * far the prediction could be out, and it produces the route. Nothing
 * about it assumes where the datum came from.
 */
export function planSearch(spec: SearchSpec): Waypoint[] {
  const clean: SearchSpec = {
    ...spec,
    radiusM: Math.max(spec.spacingM, spec.radiusM),
    spacingM: Math.max(2, spec.spacingM),
    altM: clampAlt(spec.altM),
  };
  const wps =
    clean.pattern === "expanding-square"
      ? expandingSquare(clean)
      : clean.pattern === "sector"
        ? sector(clean)
        : parallelTrack(clean);
  return numberWaypoints(wps);
}

export function clampAlt(altM: number): number {
  return Math.min(LIMITS.maxAltM, Math.max(LIMITS.minAltM, altM));
}

/** Renumber pins 1..n, leaving named ones (Datum) alone. */
export function numberWaypoints(wps: Waypoint[]): Waypoint[] {
  let n = 0;
  return wps.map((w) => {
    n += 1;
    return w.label && !/^\d+$/.test(w.label) ? { ...w, label: `${n} ${w.label}` } : { ...w, label: String(n) };
  });
}

// --------------------------------------------------------------------
// Checking a plan before anything is sent
// --------------------------------------------------------------------

export interface PlanReview {
  legs: number;
  /** Route length in metres, including the run out from `from`. */
  totalM: number;
  /** At the guidance's cruise speed, seconds. Ignores climb and turns. */
  etaS: number;
  /** Longest single leg - the one the range limit will reject first. */
  longestLegM: number;
  /** Furthest any waypoint sits from the start point. */
  furthestM: number;
  errors: string[];
  warnings: string[];
}

/**
 * Everything worth knowing before pressing fly, with the problems split
 * into ones that WILL be refused and ones that are merely worth seeing.
 *
 * `from` is where the aircraft is now. It matters: mission.py measures
 * its range limit from the drone's CURRENT position each time a target is
 * sent, so a route can be legal leg by leg and still strand itself if it
 * walks away in 299 m hops. The furthest-from-start figure is what tells
 * you that.
 */
export function reviewPlan(waypoints: Waypoint[], from: LatLon | null): PlanReview {
  const errors: string[] = [];
  const warnings: string[] = [];
  let totalM = 0;
  let longestLegM = 0;
  let furthestM = 0;

  if (waypoints.length === 0) {
    errors.push("The plan is empty - drop a pin or generate a search pattern.");
  }

  let prev: LatLon | null = from;
  for (const w of waypoints) {
    if (prev) {
      const d = distanceM(prev, w);
      totalM += d;
      longestLegM = Math.max(longestLegM, d);
      if (d > LIMITS.maxRangeM) {
        errors.push(
          `Leg to ${w.label ?? "a waypoint"} is ${Math.round(d)} m. The aircraft refuses anything over ${LIMITS.maxRangeM} m.`,
        );
      }
    }
    if (from) furthestM = Math.max(furthestM, distanceM(from, w));
    if (w.altM < LIMITS.minAltM || w.altM > LIMITS.maxAltM) {
      errors.push(
        `${w.label ?? "A waypoint"} is set to ${w.altM} m. The accepted band is ${LIMITS.minAltM}-${LIMITS.maxAltM} m.`,
      );
    }
    prev = w;
  }

  if (!from) {
    warnings.push(
      "No GPS fix yet, so range cannot be checked. Every leg is still checked by the aircraft when it is sent.",
    );
  } else if (furthestM > LIMITS.maxRangeM) {
    warnings.push(
      `The plan reaches ${Math.round(furthestM)} m from where the drone is now. Each leg is measured from wherever it has got to, so this is allowed - but it walks the aircraft beyond ${LIMITS.maxRangeM} m from here.`,
    );
  }

  const etaS = totalM / LIMITS.cruiseMs;
  if (etaS > 8 * 60) {
    warnings.push(
      `About ${Math.round(etaS / 60)} minutes of flying at ${LIMITS.cruiseMs} m/s. Check the pack will last it - the low-battery failsafe will interrupt the search if not.`,
    );
  }

  return {
    legs: Math.max(0, waypoints.length - (from ? 0 : 1)),
    totalM,
    etaS,
    longestLegM,
    furthestM,
    errors,
    warnings,
  };
}

/** Portable form, so a plan can be saved, mailed, or replayed later. */
export function exportPlan(waypoints: Waypoint[], spec?: SearchSpec | null): string {
  return JSON.stringify(
    {
      kind: "seayou.flightplan",
      version: 1,
      created: new Date().toISOString(),
      search: spec ?? null,
      waypoints: waypoints.map(({ lat, lon, altM, label }) => ({ lat, lon, altM, label })),
    },
    null,
    2,
  );
}

/**
 * Read a plan back. Returns null rather than throwing, and rejects
 * anything whose coordinates are not real - a plan file is a thing that
 * flies an aircraft, so it gets checked like one.
 */
export function importPlan(text: string): Waypoint[] | null {
  try {
    const d = JSON.parse(text);
    const raw = Array.isArray(d) ? d : d?.waypoints;
    if (!Array.isArray(raw) || raw.length === 0) return null;
    const out: Waypoint[] = [];
    for (const w of raw) {
      const lat = Number(w?.lat);
      const lon = Number(w?.lon);
      const altM = Number(w?.altM ?? w?.alt ?? 5);
      if (!Number.isFinite(lat) || Math.abs(lat) > 90) return null;
      if (!Number.isFinite(lon) || Math.abs(lon) > 180) return null;
      if (!Number.isFinite(altM)) return null;
      out.push(waypoint(lat, lon, clampAlt(altM), typeof w?.label === "string" ? w.label : undefined));
    }
    return numberWaypoints(out);
  } catch {
    return null;
  }
}
