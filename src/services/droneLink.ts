/**
 * SeaYou drone link - the single connection this app has to the drone.
 *
 * ---------------------------------------------------------------------
 * THIS FILE FLIES THE DRONE. Read this header before changing anything.
 * ---------------------------------------------------------------------
 *
 * The flight-control logic here is a deliberate PORT of the drone's own
 * pilot dashboard (`Raspberry Pi 5/dashboard/static/dashboard.js`), which
 * is the version that has actually been flown and bench-tested. Every
 * constant below - the 30 Hz send rate, the 25 %/s throttle ramp, the
 * 0.22 axis lock, the exact byte mapping - is identical to that file on
 * purpose. If you change one here, change it there too, and re-test with
 * the props OFF before flying. A number that disagrees between the two
 * dashboards is a bug waiting to happen in the air.
 *
 * Why this is a singleton, and not a hook or a per-component object:
 * the Pi's server keeps only ONE shared `_latest_control` across every
 * connected client - last write wins (see PicoLink.set_control in
 * server.py). Two components each opening a socket and each sending
 * control would fight over the throttle 30 times a second. There is one
 * drone, so there is one link and one sender.
 *
 * That same server-side detail is why sending is OPT-IN (`takeControl`).
 * While control is released this module is purely a telemetry listener,
 * exactly as the dashboard was before, and is safe to leave open next to
 * the pilot dashboard. If it sent disarmed throttle-0 packets the whole
 * time, it would intermittently stamp on the real pilot's throttle and
 * drop the drone out of the sky.
 */

// ---------------------------------------------------------------------
// Where the drone is.
//
// The Pi is on DHCP, so a hardcoded IP goes stale the moment it joins a
// different network - which is exactly what happens when you demo on a
// phone hotspot instead of the lab WiFi. mDNS ("drone.local") follows it
// automatically and is the normal case.
//
// The two overrides exist because a broken address at a demo is not the
// time for a rebuild:
//   1. ?drone=192.168.0.50 in the URL - instant, no rebuild, and it
//      sticks for the rest of the session.
//   2. localStorage "droneHost" - survives a reload.
// ---------------------------------------------------------------------

const DEFAULT_DRONE_HOST = "drone.local";

/**
 * An explicitly requested drone address, or null for the normal case.
 * Setting one deliberately BYPASSES the dev-server proxy and talks to
 * the drone directly, which is what makes it a usable escape hatch when
 * the proxy's own target is the thing that is broken.
 */
function resolveExplicitHost(): string | null {
  try {
    const fromUrl = new URLSearchParams(window.location.search).get("drone");
    if (fromUrl) {
      window.localStorage.setItem("droneHost", fromUrl);
      return fromUrl;
    }
    return window.localStorage.getItem("droneHost") || null;
  } catch {
    // Private mode, or no window at all - fall back to the normal case.
    return null;
  }
}

const explicitHost = resolveExplicitHost();

export const DRONE_HOST = explicitHost ?? DEFAULT_DRONE_HOST;
export const DRONE_PORT = 8080;

export type LinkMode = "direct" | "proxy" | "same-origin";

/**
 * Three ways this app can reach the drone, in priority order:
 *
 *  "direct"      - an explicit ?drone=<host>. Talks straight to that
 *                  address, bypassing everything else. The escape hatch
 *                  for when the normal route is the broken thing.
 *  "proxy"       - `npm run dev` on a laptop. The browser talks to the
 *                  Vite dev server and it forwards to the drone (see the
 *                  /drone proxy in vite.config.ts), so a phone never has
 *                  to resolve "drone.local" - which Android often cannot.
 *  "same-origin" - the built app, served BY the Pi itself at
 *                  http://drone.local:8080. The page and the drone are
 *                  then the same server, so plain relative paths are
 *                  correct and work just as well over a raw IP.
 */
export const linkMode: LinkMode = explicitHost
  ? "direct"
  : import.meta.env.DEV
    ? "proxy"
    : "same-origin";

const wsProto = window.location.protocol === "https:" ? "wss:" : "ws:";

const DRONE_BASE =
  linkMode === "direct"
    ? `http://${DRONE_HOST}:${DRONE_PORT}`
    : linkMode === "proxy"
      ? `${window.location.origin}/drone`
      : ""; // same-origin: relative paths like "/stream.mjpg"

const DRONE_WS_URL =
  linkMode === "direct"
    ? `ws://${DRONE_HOST}:${DRONE_PORT}/ws`
    : linkMode === "proxy"
      ? `${wsProto}//${window.location.host}/drone/ws`
      : `${wsProto}//${window.location.host}/ws`;

export const droneVideoStreamUrl = `${DRONE_BASE}/stream.mjpg`;

/** Point the app at a different drone address and reconnect. */
export function setDroneHost(host: string) {
  try {
    window.localStorage.setItem("droneHost", host.trim());
  } catch {
    /* private mode - the reload below still picks up the URL form */
  }
  window.location.reload();
}

// ---------------------------------------------------------------------
// Telemetry shapes (unchanged from the original read-only service).
// ---------------------------------------------------------------------

export interface DroneAttitude {
  roll: number;
  pitch: number;
  yaw: number;
}

/**
 * Position. Since the u-blox USB dongle replaced the Pico's dead GPS
 * module this comes from gps.py on the Pi, not from the telemetry packet,
 * and `source` says which: "pi_usb" is the dongle.
 *
 * `lat`/`lon` are null whenever there is no fix, and that is the only
 * honest reading of "we do not know where the drone is". Never fall back
 * to 0,0 - that is a real place in the Gulf of Guinea, and a map that
 * shows the aircraft there looks exactly like a working map.
 */
export interface DroneGps {
  fix: string | null;
  has_fix: boolean;
  sat_count: number;
  /** Decimal degrees, or null with no fix. */
  lat?: number | null;
  lon?: number | null;
  /** Raw NMEA fields - only present on the old Pico-sourced packets. */
  lat_raw?: string | null;
  lon_raw?: string | null;
  /** "pi_usb" for the dongle; absent on Pico-sourced GPS. */
  source?: string;
  /** Metres above mean sea level, from GGA. */
  alt_m?: number | null;
  /** Satellites visible vs actually used in the fix (sat_count). */
  sats_in_view?: number;
  /** Best carrier-to-noise of any tracked satellite, dB-Hz. */
  best_snr?: number;
  hdop?: number | null;
  /** Ground speed, m/s, from RMC. */
  speed_ms?: number | null;
  /** Course over ground, degrees true. Null below 0.5 m/s - it is noise. */
  course_deg?: number | null;
  /** Why there is no fix: "acquiring" | "stale" | "serial_error". */
  state?: string;
  ts?: number;
}

export interface DroneBaro {
  temp_c: number;
  pressure_hpa: number;
  height_m: number;
}

/**
 * The Pico's stored level zero-point - what the aircraft believes "flat"
 * is. Null on firmware built before Level_Capture_Run(), in which case
 * the panel hides rather than showing offsets that come from nowhere.
 */
export interface DroneLevel {
  state: "idle" | "settling" | "sampling" | "unknown";
  busy: boolean;
  result: "ok" | "moved" | "out_of_range" | "throttle_not_zero" | null;
  roll_offset_deg: number;
  pitch_offset_deg: number;
}

/**
 * Pack voltage from the INA219 on the Pi's own I2C bus.
 *
 * `voltage_v` is null whenever the reading cannot be trusted, and `fault`
 * says why. That distinction is the whole point: a battery monitor that
 * reports 0.0 V for a wiring problem looks exactly like a flat pack, which
 * is the one mistake this must not make. Render the fault, never a number
 * you were not given.
 *
 *   open_shunt  VIN- is not connected - the chip never sees the pack
 *   overflow    the INA219's own math-overflow flag
 *   i2c_error   the bus read failed
 */
export interface DroneBattery {
  voltage_v: number | null;
  current_a?: number | null;
  power_w?: number | null;
  shunt_mv?: number;
  cells?: number;
  cell_v?: number;
  /**
   * pack_across_shunt is the hazardous one: the battery is wired across
   * VIN+ and VIN-, i.e. straight across the module's 0.1 ohm shunt. Bus
   * voltage is measured AT VIN-, which in that wiring is battery
   * negative, so it reads 0 V and always will.
   */
  fault?: "pack_across_shunt" | "no_pack" | "open_shunt" | "overflow" | "i2c_error";
  /** Short instruction from the Pi for the current fault, if any. */
  hint?: string;
  /** e.g. "voltage only - shunt not in a current path". */
  note?: string;
  i2c_faults?: number;
}

/**
 * VSYS - the drone's 5 V rail, measured by the Pico's own ADC through its
 * internal 3:1 divider. No extra hardware involved, so this reading works
 * whatever state the INA219 is in.
 *
 * Per the V4 wiring diagram this is the same rail that feeds the GPS
 * module's VCC and the I2C sensors, which makes it more than a Pico health
 * figure: it says whether those parts have a supply at all.
 *
 *   usb_backfeed  ~4.8 V - only USB leaking through the Pico's Schottky.
 *                 The ESC BEC is not supplying the rail.
 *   esc_bec       >= 4.95 V - the BEC is genuinely powering it.
 */
export interface DroneRail {
  vsys_v: number;
  source: "usb_backfeed" | "esc_bec";
}

/**
 * The on-board low-pack failsafe (battery_guard.py on the Pi).
 *
 * Runs entirely on the drone and needs no ground station, so these levels are
 * a REPORT of what the aircraft has already decided - not a request for the
 * dashboard to do something. Levels only ever escalate within a flight.
 *
 *   ok        nothing to do
 *   warn      finish up and land soon; nothing automatic has happened
 *   return    flying home on its own (or landing, with no GPS fix)
 *   land_now  coming down where it is, immediately
 */
export interface DroneBatteryGuard {
  level: "unknown" | "ok" | "warn" | "return" | "land_now";
  reason: string;
  cell_v: number | null;
  pack_v: number | null;
  cells: number;
  thresholds: { warn: number; return: number; land_now: number };
  /**
   * The distance-aware return reserve (battery_guard.py).
   *
   * The aircraft holds back enough pack to fly home from wherever it
   * currently is, so the further out it goes the earlier it turns back.
   * `headroom_v` is what is left before it does that on its own -
   * negative means it already has. All-null means the reserve is not
   * active (no fix or no home recorded) and the fixed threshold is in
   * charge instead.
   */
  reserve?: {
    distance_home_m: number | null;
    time_home_s: number | null;
    reserve_v: number;
    return_at_v: number;
    headroom_v: number | null;
    fall_v_per_min: number | null;
    measured: boolean;
  } | null;
}

/** What each refusal from the firmware actually means. */
export const LEVEL_RESULT_TEXT: Record<string, string> = {
  ok: "Captured and saved",
  moved: "Aborted - the drone moved",
  out_of_range: "Aborted - more than 15\u00b0 out, check the mounting",
  throttle_not_zero: "Aborted - throttle must be at zero",
};

/**
 * Autonomous flight, computed ON the Pi and relayed on. Present when the
 * link runs through the ground station; absent talking straight to the
 * aircraft, so every field is optional at the use site.
 */
export interface MissionStatus {
  state: "idle" | "running" | "holding" | "arrived" | "aborted";
  reason: string;
  target: { lat: number; lon: number } | null;
  target_alt_m: number;
  distance_m: number | null;
  bearing_deg: number | null;
  elapsed_s: number;
  interrupted_by_detection?: boolean;
}

/** The on-board detector's state, same provenance as MissionStatus. */
export interface DetectionStatus {
  enabled: boolean;
  ready: boolean;
  error: string | null;
  confirmed: boolean;
  frames: number;
  fps: number | null;
}

export interface DroneTelemetry {
  attitude: DroneAttitude;
  mission?: MissionStatus;
  detection?: DetectionStatus;
  gps: DroneGps;
  baro: DroneBaro | null;
  /** Per-motor output 0-1000 the Pico actually computed. */
  motor_out_debug?: number[];
  /** False means the boot-time gyro bias sweep was rejected - do not fly. */
  gyro_cal_ok?: boolean | null;
  /** The stored level zero-point, or null on older firmware. */
  level?: DroneLevel | null;
  /** Pack voltage, or absent when no INA219 is fitted. */
  battery?: DroneBattery | null;
  /** What the on-board low-pack failsafe has decided. */
  battery_guard?: DroneBatteryGuard | null;
  /** The 5 V rail, from the Pico's ADC. Absent on 79-byte firmware. */
  rail?: DroneRail | null;
  ts: number;
}

export type DroneLinkState = "connecting" | "connected" | "disconnected";

// ---------------------------------------------------------------------
// Control constants - PORTED VERBATIM from dashboard.js. See header.
// ---------------------------------------------------------------------

/** Control packets per second. Matches the pilot dashboard. */
const SEND_HZ = 30;

/**
 * Throttle is RATE-controlled, not position-controlled.
 *
 * Holding the throttle stick off-centre RAMPS the throttle; the stick
 * springs back to centre on release and the throttle HOLDS wherever it
 * got to. That is what allows a hover - a position-controlled stick
 * would drop the drone the instant you let go.
 *
 * The consequence matters and is not optional: RELEASING THE STICK DOES
 * NOT STOP THE MOTORS. Disarming is the only thing that does.
 */
const THROTTLE_RATE = 25; // percent of full throttle per second, at full deflection

/** Ignore residual deflection so the throttle never creeps on its own. */
const THROTTLE_DEADZONE = 0.05;

/**
 * Left-stick axis lock. Past this much deflection the stick commits to
 * whichever axis you actually pushed and zeroes the other.
 *
 * Sideways on the left stick is YAW, and the mixer turns yaw into one
 * diagonal motor pair outrunning the other. Yaw leaking into a climb is
 * exactly the failure this prevents. The right stick is deliberately NOT
 * locked - roll and pitch together is normal flying.
 */
const LOCK_AT = 0.22;

/**
 * Pitch sign convention, matching Tx_Rx_Update_Variables() on the Pico.
 * Verify with props OFF before arming with props on, and flip this if
 * the drone pitches the wrong way.
 */
const PITCH_INVERTED = false;

/** Trim limit in degrees. Well inside the firmware's own range. */
const TRIM_LIMIT_DEG = 12;

export interface ControlPacket {
  roll: number;
  pitch: number;
  throttle: number;
  yaw: number;
  cmd0: number;
  cmd1: number;
  roll_trim: number;
  pitch_trim: number;
}

// ---------------------------------------------------------------------
// Gamepad mapping.
//
// These are the "standard" gamepad indices, which Chrome reports for a
// DualShock 4 and a DualSense over both USB and Bluetooth. A third-party
// or oddly-paired pad may number things differently - the CONTROLLER
// panel on the Live Feeds page shows the live axis values and the index
// of whatever button you press, so you can read off the real numbers and
// correct these lines without guessing.
// ---------------------------------------------------------------------

const GP_AXIS_LEFT_X = 0; // yaw
const GP_AXIS_LEFT_Y = 1; // throttle ramp   (browser reports -1 for UP)
const GP_AXIS_RIGHT_X = 2; // roll
const GP_AXIS_RIGHT_Y = 3; // pitch          (browser reports -1 for UP)

/**
 * EMERGENCY STOP - both shoulder buttons, either one on its own.
 * Two of them because in a panic you should not have to find one
 * specific finger, and both shoulders are reachable without looking away
 * from the drone. Instant, no hold: this is the stop, it must never
 * need a technique.
 */
const GP_BTN_DISARM_A = 4; // L1
const GP_BTN_DISARM_B = 5; // R1

const GP_BTN_ARM = 9; // Options - must be HELD, see GP_ARM_HOLD_MS

/**
 * Trim on the four face buttons, laid out the way they physically sit on
 * the pad, so the button you press points the way the drone should go:
 * Triangle on top = nose forward, Cross underneath = nose back,
 * Square on the left = roll left, Circle on the right = roll right.
 */
const GP_BTN_TRIM_PITCH_FWD = 3; // Triangle
const GP_BTN_TRIM_PITCH_BACK = 0; // Cross
const GP_BTN_TRIM_ROLL_LEFT = 2; // Square
const GP_BTN_TRIM_ROLL_RIGHT = 1; // Circle

// The D-pad does the same four trims, kept as a redundant alternative.
const GP_BTN_DPAD_UP = 12; // pitch trim forward
const GP_BTN_DPAD_DOWN = 13; // pitch trim back
const GP_BTN_DPAD_LEFT = 14; // roll trim left
const GP_BTN_DPAD_RIGHT = 15; // roll trim right

/**
 * Arming needs a deliberate HOLD so a bumped button can never spin the
 * props up. Disarming is the exact opposite - a single instant press,
 * because it is the emergency stop and must never need a technique.
 */
const GP_ARM_HOLD_MS = 600;

/**
 * A physical thumbstick almost never rests at exactly zero; a well-used
 * pad can sit 0.05-0.15 off centre. That matters far more here than on a
 * touchscreen, because a resting offset on the LEFT stick's Y axis would
 * ramp the throttle up on its own with nobody touching anything.
 *
 * Radial (measured on the stick's distance from centre rather than
 * per-axis) so a round stick gate feels even, and rescaled so travel just
 * past the deadzone starts from zero instead of jumping to 0.15.
 */
const GP_DEADZONE = 0.08;

/**
 * Adaptive jitter filter for the sticks.
 *
 * A worn stick reports small random wobble even when nobody is touching
 * it, which showed up as a twitching dot and twitching output. A plain
 * low-pass would fix that and make the controls feel laggy, which is the
 * opposite of what is wanted here.
 *
 * So the smoothing is proportional to how fast the stick is actually
 * moving: tiny frame-to-frame changes are almost certainly noise and get
 * smoothed hard, while a deliberate sweep passes through untouched. The
 * result is a stick that sits dead still at rest and still responds
 * instantly when you move it.
 *
 * MIN_ALPHA - smoothing floor, applied to the smallest movements.
 * FULL_AT   - frame-to-frame change at which filtering stops entirely.
 *             Noise measured on this pad is well under 0.03/frame; a fast
 *             deliberate sweep is around 0.1/frame, so 0.08 separates them.
 *
 * The deadzone above could also be dropped from 0.15 to 0.08 BECAUSE of
 * this filter - the filter now handles the noise the deadzone used to
 * have to swallow, which gives back nearly half the stick's usable travel
 * around centre and is most of why the controls feel sharper.
 */
const JITTER_MIN_ALPHA = 0.1;
const JITTER_FULL_AT = 0.08;

/**
 * How many samples the auto-centre averages over. The poll runs on
 * requestAnimationFrame, so ~60 samples is about a second.
 */
const GP_AUTOCENTRE_SAMPLES = 90;

/**
 * How much an axis may wander ACROSS that whole window and still count as
 * "resting".
 *
 * This started at 0.03 and that was wrong: this pad's own sensor noise is
 * 0.06 on roll and 0.08 on pitch while genuinely untouched, so the window
 * never completed and the centre was never captured - the drift went
 * uncorrected with nothing on screen to say why. It has to sit above the
 * pad's noise floor but well below deliberate movement, which swings by
 * whole tenths.
 */
const GP_AUTOCENTRE_TOLERANCE = 0.1;

/**
 * A window whose start and end differ by more than this is a stick being
 * moved SLOWLY, not one at rest, and must not be captured as a centre.
 *
 * This is the bug that broke roll. The spread test alone passes happily
 * during a slow sweep - drift 0.09 across the window and every sample is
 * still "within tolerance" - so the filter captured a centre of -0.276 on
 * a stick that actually rests at +0.082. Everything downstream then had
 * neutral in the wrong place, which is what made roll-right feel dead and
 * roll-left twitchy.
 */
const GP_AUTOCENTRE_MAX_TREND = 0.04;

/**
 * Largest offset accepted as genuine stick drift. Measured resting values
 * on this pad are all under 0.09; anything past this is a held stick.
 * Was 0.8, which was far too generous - it let the bad -0.276 straight
 * through.
 */
const GP_AUTOCENTRE_MAX_OFFSET = 0.35;

/** Median of a numeric list. Used instead of the mean because it ignores
 *  a few outlying samples rather than letting them drag the centre. */
function medianOf(v: number[]): number {
  const s = [...v].sort((a, b) => a - b);
  const m = s.length >> 1;
  return s.length % 2 ? s[m] : (s[m - 1] + s[m]) / 2;
}

export const GP_BUTTON_NAMES: Record<number, string> = {
  0: "Cross",
  1: "Circle",
  2: "Square",
  3: "Triangle",
  4: "L1",
  5: "R1",
  6: "L2",
  7: "R2",
  8: "Share",
  9: "Options",
  10: "L3",
  11: "R3",
  12: "D-pad up",
  13: "D-pad down",
  14: "D-pad left",
  15: "D-pad right",
  16: "PS",
};

export interface GamepadInfo {
  id: string;
  /** True when the browser recognised the pad and GP_BUTTON_NAMES applies. */
  standard: boolean;
  /** Post-deadzone stick positions, +y = up, matching the rest of this file. */
  lx: number;
  ly: number;
  rx: number;
  ry: number;
  /** Raw axis values straight from the browser, for the mapping panel. */
  rawAxes: number[];
  /** Resting offsets currently being subtracted. All zero = uncalibrated. */
  centre: number[];
  /**
   * Lowest and highest RAW value seen on each axis since the pad
   * connected. Sweeping a stick to both extremes fills these in, which is
   * how you tell a stick that merely rests off-centre (full -1..+1 travel,
   * fixable by centring) from one whose electrical zero has moved to the
   * end of its range (truncated travel - a failing potentiometer, not
   * something software can correct).
   */
  rangeMin: number[];
  rangeMax: number[];
  /** True once a non-zero centre has been captured. */
  centred: boolean;
  lastButton: number | null;
  /** 0..1 while the ARM button is being held down. */
  armHoldProgress: number;
}

// ---------------------------------------------------------------------
// The public snapshot the UI renders from.
// ---------------------------------------------------------------------

export interface DroneSnapshot {
  linkState: DroneLinkState;
  telemetry: DroneTelemetry | null;
  packetAgeSeconds: number | null;
  /**
   * Is the AIRCRAFT on the other end? Different question from linkState:
   * through the ground station the socket can be perfectly healthy while
   * the drone is switched off. True by default, because connected
   * straight to the aircraft the socket is the aircraft.
   */
  droneConnected: boolean;
  /** Is this app sending control packets at all? Opt-in - see header. */
  hasControl: boolean;
  armed: boolean;
  /** Held throttle 0-100. Survives releasing the stick; only disarm clears it. */
  throttle: number;
  rollTrim: number;
  pitchTrim: number;
  gamepad: GamepadInfo | null;
  /**
   * The control bytes as they would go on the wire right now: 50 is
   * neutral for roll/pitch/yaw, 0 is motors-off for throttle.
   *
   * Shown in the UI because the raw axis numbers jitter constantly on a
   * worn stick even while the output is a clean, dead-centre 50 - so
   * watching the raw values makes a perfectly healthy setup look like it
   * is drifting. This is the number that actually matters.
   */
  command: { roll: number; pitch: number; yaw: number; throttle: number };
  /** Why the last automatic disarm happened, for the UI to explain itself. */
  lastEvent: string | null;
  /**
   * Sticky feedback from the last SET LEVEL / ERASE press. The firmware
   * answers asynchronously, and a capture takes three seconds, so without
   * this the button appears to do nothing at all.
   */
  levelMessage: string | null;
}

type Listener = () => void;

// ---------------------------------------------------------------------

function clamp(v: number, lo: number, hi: number) {
  return Math.max(lo, Math.min(hi, v));
}

/** Round for snapshot comparison so a still stick does not re-render at 30 Hz. */
function q(v: number) {
  return Math.round(v * 100) / 100;
}

class DroneLink {
  private ws: WebSocket | null = null;
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null;
  private sendTimer: ReturnType<typeof setTimeout> | null = null;
  private rafId: number | null = null;
  private ageTimer: ReturnType<typeof setInterval> | null = null;

  /** How many React components currently want the link alive. */
  private refs = 0;

  private listeners = new Set<Listener>();

  // --- link state ---
  private linkState: DroneLinkState = "connecting";
  private telemetry: DroneTelemetry | null = null;
  private packetAgeSeconds: number | null = null;

  // --- control state (mirrors `state` in dashboard.js) ---
  private hasControl = false;
  private armed = false;
  private throttle = 0;
  private rollTrim = 0;
  private pitchTrim = 0;
  private left = { x: 0, y: 0 }; // yaw, throttle
  private right = { x: 0, y: 0 }; // roll, pitch
  private lastSendAt = 0;
  /** Rate limiter for the stick-diagnostics message. See sendLoop(). */
  private lastGpDebugAt = 0;
  /** Running output of the jitter filter, one per axis. */
  private gpFiltered: number[] = [0, 0, 0, 0];
  private lastEvent: string | null = null;
  private levelMessage: string | null = null;
  private levelMessageAt = 0;

  // --- gamepad state ---
  private gpIndex: number | null = null;
  private gpInfo: GamepadInfo | null = null;
  private gpLockedAxis: "x" | "y" | null = null;
  private gpPrevButtons: boolean[] = [];
  private gpArmHeldSince = 0;
  private gpArmFired = false;
  /** Last raw axis values straight from the browser, pre-calibration. */
  private gpRawAxes: number[] = [];
  /**
   * Per-axis resting offset, subtracted before anything else looks at the
   * sticks. A worn thumbstick does not return to zero - this pad's right
   * stick X sits near -0.55 untouched, which would command a hard roll
   * the instant it armed. Widening the deadzone to swallow that would
   * throw away more than half the stick's travel, so the offset is
   * measured and subtracted instead.
   */
  private gpCentre: number[] = [0, 0, 0, 0];
  /** Auto-centring runs once per pad connection. See maybeAutoCentre(). */
  private gpAutoCentred = false;
  private gpSteadyWindow: number[][] = [];
  private gpAxisMin: number[] = [];
  private gpAxisMax: number[] = [];
  /** Screen wake lock, held only while armed. See acquireWakeLock(). */
  private wakeLock: { release: () => Promise<void> } | null = null;
  /** See DroneSnapshot.droneConnected. Only the ground station moves it. */
  private droneConnected = true;

  private snapshot: DroneSnapshot = {
    linkState: "connecting",
    telemetry: null,
    packetAgeSeconds: null,
    droneConnected: true,
    hasControl: false,
    armed: false,
    throttle: 0,
    rollTrim: 0,
    pitchTrim: 0,
    gamepad: null,
    command: { roll: 50, pitch: 50, yaw: 50, throttle: 0 },
    lastEvent: null,
    levelMessage: null,
  };

  constructor() {
    // Restore trim the same way the pilot dashboard does, so the drone
    // behaves identically whichever dashboard you fly it from.
    try {
      const r = parseFloat(window.localStorage.getItem("rollTrim") ?? "");
      const p = parseFloat(window.localStorage.getItem("pitchTrim") ?? "");
      if (!Number.isNaN(r)) this.rollTrim = clamp(r, -TRIM_LIMIT_DEG, TRIM_LIMIT_DEG);
      if (!Number.isNaN(p)) this.pitchTrim = clamp(p, -TRIM_LIMIT_DEG, TRIM_LIMIT_DEG);
      const c = JSON.parse(window.localStorage.getItem("stickCentre") ?? "null");
      // Same sanity cap as a fresh capture. A previously stored centre can
      // be wrong - an earlier version of the auto-centre latched -0.276 on
      // a stick that rests at +0.082 - and a bad stored value would other-
      // wise persist across reloads and keep neutral in the wrong place.
      // Dropping it here means the next good capture simply replaces it.
      if (
        Array.isArray(c) &&
        c.length === 4 &&
        c.every((v) => typeof v === "number" && Math.abs(v) <= GP_AUTOCENTRE_MAX_OFFSET)
      ) {
        this.gpCentre = c;
      } else if (c) {
        window.localStorage.removeItem("stickCentre");
      }
    } catch {
      /* private mode - start at zero trim and an uncalibrated stick */
    }
    this.publish();
  }

  // -------------------------------------------------------------------
  // Subscription. Refcounted because React StrictMode mounts every
  // effect twice in development - without this the second mount would
  // tear down the socket the first one just opened.
  // -------------------------------------------------------------------

  subscribe = (fn: Listener): (() => void) => {
    this.listeners.add(fn);
    this.refs += 1;
    if (this.refs === 1) this.start();
    return () => {
      this.listeners.delete(fn);
      this.refs -= 1;
      if (this.refs === 0) this.stop();
    };
  };

  getSnapshot = (): DroneSnapshot => this.snapshot;

  private publish() {
    const next: DroneSnapshot = {
      linkState: this.linkState,
      telemetry: this.telemetry,
      packetAgeSeconds: this.packetAgeSeconds,
      droneConnected: this.droneConnected,
      hasControl: this.hasControl,
      armed: this.armed,
      throttle: Math.round(this.throttle),
      rollTrim: this.rollTrim,
      pitchTrim: this.pitchTrim,
      gamepad: this.gpInfo,
      // Expires on its own: a four-second-old "Capture requested" sitting
      // under a finished capture is worse than no message.
      levelMessage:
        this.levelMessage && performance.now() - this.levelMessageAt < 6000
          ? this.levelMessage
          : null,
      command: (() => {
        const c = this.buildControl();
        return { roll: c.roll, pitch: c.pitch, yaw: c.yaw, throttle: c.throttle };
      })(),
      lastEvent: this.lastEvent,
    };

    // Only hand React a new object when something it can actually see
    // changed. The send loop runs at 30 Hz and the gamepad poll at 60 Hz;
    // without this, a drone sitting still would re-render the whole page
    // ninety times a second for nothing.
    const prev = this.snapshot;
    const same =
      prev.linkState === next.linkState &&
      prev.telemetry === next.telemetry &&
      prev.packetAgeSeconds === next.packetAgeSeconds &&
      prev.hasControl === next.hasControl &&
      prev.armed === next.armed &&
      prev.throttle === next.throttle &&
      prev.rollTrim === next.rollTrim &&
      prev.pitchTrim === next.pitchTrim &&
      prev.lastEvent === next.lastEvent &&
      prev.levelMessage === next.levelMessage &&
      prev.command.roll === next.command.roll &&
      prev.command.pitch === next.command.pitch &&
      prev.command.yaw === next.command.yaw &&
      prev.command.throttle === next.command.throttle &&
      sameGamepad(prev.gamepad, next.gamepad);
    if (same) return;

    this.snapshot = next;
    this.listeners.forEach((fn) => fn());
  }

  // -------------------------------------------------------------------
  // Lifecycle
  // -------------------------------------------------------------------

  private start() {
    this.connect();
    this.lastSendAt = performance.now();
    this.sendLoop();
    this.pollGamepad();

    this.ageTimer = setInterval(() => {
      this.packetAgeSeconds = this.telemetry ? Date.now() / 1000 - this.telemetry.ts : null;
      this.publish();
    }, 1000);

    window.addEventListener("blur", this.onBlur);
    window.addEventListener("keydown", this.onKeyDown);
    document.addEventListener("visibilitychange", this.onVisibility);
    window.addEventListener("gamepadconnected", this.onGamepadConnected);
    window.addEventListener("gamepaddisconnected", this.onGamepadDisconnected);
  }

  private stop() {
    // Releasing control on teardown is not optional: if the page is going
    // away, the drone must not be left armed with a latched throttle.
    this.disarm("dashboard closed");
    this.hasControl = false;

    if (this.reconnectTimer) clearTimeout(this.reconnectTimer);
    if (this.sendTimer) clearTimeout(this.sendTimer);
    if (this.ageTimer) clearInterval(this.ageTimer);
    if (this.rafId !== null) cancelAnimationFrame(this.rafId);
    this.reconnectTimer = this.sendTimer = this.ageTimer = null;
    this.rafId = null;

    window.removeEventListener("blur", this.onBlur);
    window.removeEventListener("keydown", this.onKeyDown);
    document.removeEventListener("visibilitychange", this.onVisibility);
    window.removeEventListener("gamepadconnected", this.onGamepadConnected);
    window.removeEventListener("gamepaddisconnected", this.onGamepadDisconnected);

    const ws = this.ws;
    this.ws = null;
    // Drop the handlers first so the close does not schedule a reconnect
    // for a link nobody is watching any more.
    if (ws) {
      ws.onopen = ws.onclose = ws.onerror = ws.onmessage = null;
      ws.close();
    }
  }

  // -------------------------------------------------------------------
  // WebSocket
  // -------------------------------------------------------------------

  private connect = () => {
    if (this.refs === 0) return;
    this.linkState = "connecting";
    this.publish();

    const ws = new WebSocket(DRONE_WS_URL);
    this.ws = ws;

    ws.onopen = () => {
      if (this.ws !== ws) return;
      this.linkState = "connected";
      this.publish();
    };

    ws.onmessage = (event) => {
      if (this.ws !== ws) return;
      try {
        const msg = JSON.parse(event.data);
        if (msg.type === "telemetry") {
          this.telemetry = msg.data;
          this.packetAgeSeconds = 0;
          this.publish();
        } else if (msg.type === "drone_status") {
          // Only the ground station sends this. Connected straight to the
          // aircraft the socket IS the aircraft, so the default below
          // stays true and nothing changes.
          this.droneConnected = Boolean(msg.data?.connected);
          this.publish();
        } else if (msg.type === "level_cal_ack") {
          // The ack only says the request reached the aircraft. Whether
          // the capture SUCCEEDED arrives later, in telemetry's level
          // block, because it takes three seconds of holding still.
          this.levelMessage = msg.data?.accepted
            ? "Capture requested - hold still"
            : `Refused: ${msg.data?.reason ?? "unknown"}`;
          this.levelMessageAt = performance.now();
          this.publish();
        }
      } catch {
        // Ignore a malformed frame rather than tearing down the link.
      }
    };

    const scheduleReconnect = () => {
      if (this.ws !== ws) return;
      // Losing the link means losing control. Disarm rather than leaving a
      // latched throttle on a drone we can no longer talk to. The Pi has
      // its own 0.5 s staleness failsafe and the Pico a 1 s one, so the
      // motors stop regardless - this just keeps the UI honest about it.
      if (this.armed) this.disarm("link lost");
      this.linkState = "disconnected";
      this.publish();
      this.reconnectTimer = setTimeout(this.connect, 2000);
    };
    ws.onclose = scheduleReconnect;
    ws.onerror = () => ws.close();
  };

  // -------------------------------------------------------------------
  // Control API
  // -------------------------------------------------------------------

  /**
   * Start or stop sending control packets.
   *
   * Off by default. While released, this app is a pure telemetry
   * listener and is safe to leave open beside the pilot dashboard.
   * While held, it is stamping on the same single `_latest_control` slot
   * the pilot dashboard writes to - so only one of them may hold control
   * at a time.
   */
  takeControl = (on: boolean) => {
    if (this.hasControl === on) return;
    if (!on) this.disarm("control released");
    this.hasControl = on;
    this.lastEvent = on ? "control taken" : "control released";
    this.publish();
  };

  /**
   * Tell the aircraft what level is, or erase what it thinks it knows.
   *
   * The firmware holds still for a second, averages for two, and refuses
   * outright if the drone moves, if the throttle is not at zero, or if it
   * is more than 15 degrees out - at which point the board is mounted
   * wrong and no amount of calibration will fix it.
   *
   * Refused while armed, for the obvious reason.
   */
  requestLevelCal = (action: "start" | "reset") => {
    if (this.armed) {
      this.levelMessage = "disarm first";
      this.levelMessageAt = performance.now();
      this.publish();
      return;
    }
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      this.levelMessage = "no link to the aircraft";
      this.levelMessageAt = performance.now();
      this.publish();
      return;
    }
    this.ws.send(JSON.stringify({ type: "level_cal", action }));
    this.levelMessage = action === "start" ? "Capture requested\u2026" : "Erase requested\u2026";
    this.levelMessageAt = performance.now();
    this.publish();
  };

  tryArm = () => {
    if (!this.hasControl) {
      this.lastEvent = "take control first";
      this.publish();
      return;
    }
    // Same gate as the pilot dashboard: the throttle stick must be at
    // rest, so arming can never immediately ramp the motors.
    if (this.left.y > 0.03) {
      this.lastEvent = "throttle must be at zero to arm";
      this.publish();
      return;
    }
    // And every OTHER axis must be at rest too. These values are already
    // past the deadzone, so a non-zero one is real deflection that would
    // go out in the very first packet after arming. This is the backstop
    // for stick drift: this pad's right stick rests near half deflection,
    // which without this gate meant arming immediately commanded a hard
    // roll. It also catches a centre captured while a stick was held.
    if (this.left.x !== 0 || this.right.x !== 0 || this.right.y !== 0) {
      this.lastEvent = "centre the sticks before arming — let go, then press CENTRE STICKS";
      this.publish();
      return;
    }
    this.armed = true;
    this.acquireWakeLock();
    this.lastEvent = "armed";
    this.publish();
  };

  disarm = (reason = "disarmed") => {
    const wasArmed = this.armed;
    this.armed = false;
    this.releaseWakeLock();
    // Drop the held throttle and centre the sticks, so re-arming always
    // starts from idle rather than resuming a hover setting.
    this.throttle = 0;
    this.left = { x: 0, y: 0 };
    this.right = { x: 0, y: 0 };
    this.gpLockedAxis = null;
    if (wasArmed) this.lastEvent = reason;
    this.publish();
  };

  toggleArm = () => {
    if (this.armed) this.disarm("disarmed");
    else this.tryArm();
  };

  adjustTrim = (axis: "roll" | "pitch", delta: number) => {
    const key = axis === "roll" ? "rollTrim" : "pitchTrim";
    this[key] = clamp(this[key] + delta, -TRIM_LIMIT_DEG, TRIM_LIMIT_DEG);
    try {
      window.localStorage.setItem(key, String(this[key]));
    } catch {
      /* private mode - trim just will not persist */
    }
    this.publish();
  };

  /**
   * Capture where the sticks actually sit when nobody is touching them,
   * and treat that as zero from now on.
   *
   * Refuses while armed, and refuses an offset beyond 0.8 - that is not
   * a worn stick, that is a stick someone is holding, and storing it
   * would permanently bias the controls in the opposite direction.
   */
  calibrateSticks = () => {
    if (this.armed) {
      this.lastEvent = "disarm before centring the sticks";
      this.publish();
      return;
    }
    const a = this.gpRawAxes;
    if (a.length < 4) {
      this.lastEvent = "no controller to centre";
      this.publish();
      return;
    }
    const offs = [a[0] ?? 0, a[1] ?? 0, a[2] ?? 0, a[3] ?? 0];
    if (offs.some((v) => Math.abs(v) > 0.8)) {
      this.lastEvent = "let go of the sticks, then centre again";
      this.publish();
      return;
    }
    this.gpCentre = offs;
    try {
      window.localStorage.setItem("stickCentre", JSON.stringify(offs));
    } catch {
      /* private mode - centring just will not persist */
    }
    this.lastEvent = "stick centre captured";
    this.publish();
  };

  resetStickCentre = () => {
    this.gpCentre = [0, 0, 0, 0];
    try {
      window.localStorage.removeItem("stickCentre");
    } catch {
      /* private mode */
    }
    this.lastEvent = "stick centre cleared";
    this.publish();
  };

  /**
   * Hold a screen wake lock while armed, so a phone does not auto-lock
   * mid-hover. That matters because hiding the page disarms - which is
   * the right behaviour, but on a phone it would otherwise fire on its
   * own after 30 seconds and drop the drone.
   *
   * This does NOT make a phone safe to fly from: a call or a notification
   * can still take focus and disarm. It only removes the timer.
   */
  private acquireWakeLock() {
    const nav = navigator as Navigator & {
      wakeLock?: { request: (t: string) => Promise<{ release: () => Promise<void> }> };
    };
    if (!nav.wakeLock?.request || this.wakeLock) return;
    nav.wakeLock
      .request("screen")
      .then((l) => {
        this.wakeLock = l;
      })
      .catch(() => {
        /* unsupported, or refused because the page was not visible */
      });
  }

  private releaseWakeLock() {
    const l = this.wakeLock;
    this.wakeLock = null;
    if (l) l.release().catch(() => undefined);
  }

  resetTrim = () => {
    this.rollTrim = 0;
    this.pitchTrim = 0;
    try {
      window.localStorage.removeItem("rollTrim");
      window.localStorage.removeItem("pitchTrim");
    } catch {
      /* private mode */
    }
    this.publish();
  };

  // -------------------------------------------------------------------
  // Failsafes. Every one of these mirrors the pilot dashboard.
  // -------------------------------------------------------------------

  private onBlur = () => this.disarm("window lost focus");

  private onVisibility = () => {
    if (document.hidden) this.disarm("tab hidden");
  };

  private onKeyDown = (e: KeyboardEvent) => {
    if (e.code === "Space") {
      e.preventDefault();
      this.disarm("spacebar");
    }
  };

  private onGamepadConnected = () => {
    this.lastEvent = "controller connected";
    this.publish();
  };

  private onGamepadDisconnected = () => {
    // A pad that runs flat or walks out of Bluetooth range mid-flight
    // must not leave the drone armed with a latched throttle and nobody
    // holding the sticks. The on-screen controls stay available as the
    // recovery path.
    this.disarm("controller disconnected");
    this.gpIndex = null;
    this.gpInfo = null;
    // Re-measure the resting position on the next connection: a different
    // pad may be plugged in, and a worn stick's rest point shifts anyway.
    this.gpAutoCentred = false;
    this.gpSteadyWindow = [];
    this.gpAxisMin = [];
    this.gpAxisMax = [];
    this.publish();
  };

  // -------------------------------------------------------------------
  // Control send loop - 30 Hz, matching the pilot dashboard.
  // -------------------------------------------------------------------

  private buildControl(): ControlPacket {
    const yaw = 50 + Math.round(this.left.x * 50);
    const roll = 50 + Math.round(this.right.x * 50);
    const pitchNorm = PITCH_INVERTED ? -this.right.y : this.right.y;
    const pitch = 50 - Math.round(pitchNorm * 50);
    return {
      roll,
      pitch,
      throttle: this.armed ? Math.round(this.throttle) : 0,
      yaw,
      cmd0: 0,
      cmd1: 0,
      // 25 is zero trim; each unit is one degree. Clamped so a runaway
      // value can never reach the wire.
      roll_trim: clamp(25 + Math.round(this.rollTrim), 25 - TRIM_LIMIT_DEG, 25 + TRIM_LIMIT_DEG),
      pitch_trim: clamp(25 + Math.round(this.pitchTrim), 25 - TRIM_LIMIT_DEG, 25 + TRIM_LIMIT_DEG),
    };
  }

  private updateThrottle(dtSeconds: number) {
    if (!this.armed) {
      this.throttle = 0;
      return;
    }
    const y = this.left.y;
    if (Math.abs(y) > THROTTLE_DEADZONE) {
      this.throttle = clamp(this.throttle + y * THROTTLE_RATE * dtSeconds, 0, 100);
    }
  }

  private sendLoop = () => {
    const now = performance.now();
    // Clamped: a backgrounded tab can stall timers for seconds, and that
    // must never arrive as one huge throttle step.
    const dt = Math.min((now - this.lastSendAt) / 1000, 0.1);
    this.lastSendAt = now;

    this.updateThrottle(dt);

    // Stick diagnostics, sent whether or not control has been taken.
    //
    // Deliberately a SEPARATE message type: it is recorded for the flight
    // log and never merged into the Pi's shared _latest_control, so it
    // cannot stamp on a real pilot's input the way a control packet would.
    // Riding it on the control packet (the first attempt) meant nothing was
    // ever captured unless TAKE CONTROL was pressed - useless for a bench
    // check, where the whole point is to test the sticks WITHOUT arming a
    // drone. 5 Hz is ample for diagnosing a stick.
    if (this.ws && this.ws.readyState === WebSocket.OPEN && this.gpRawAxes.length >= 4) {
      if (now - this.lastGpDebugAt > 200) {
        this.lastGpDebugAt = now;
        const c = this.buildControl();
        this.ws.send(
          JSON.stringify({
            type: "gp_debug",
            gp: {
              ax: this.gpRawAxes.slice(0, 4).map((v) => Math.round(v * 1000) / 1000),
              c: this.gpCentre.slice(0, 4).map((v) => Math.round(v * 1000) / 1000),
              // What those axes turned into, so the log shows the whole
              // chain: hardware -> centring -> deadzone -> wire byte.
              out: [c.roll, c.pitch, c.yaw],
            },
          })
        );
      }
    }

    if (this.hasControl && this.ws && this.ws.readyState === WebSocket.OPEN) {
      // The raw stick values ride along with every control packet so the
      // Pi can log them next to the flight data.
      //
      // This exists because a control-chain fault is otherwise undebuggable
      // after the fact: the log records ctrl_roll=50, which is equally
      // consistent with "pilot centred the stick" and "the stick is broken
      // and cannot produce anything else". Recording what the hardware
      // actually reported separates those two, and it is the only way to
      // diagnose a control problem from a flight that has already crashed.
      const gp = this.gpRawAxes.length >= 4
        ? {
            ax: this.gpRawAxes.slice(0, 4).map((v) => Math.round(v * 1000) / 1000),
            c: this.gpCentre.slice(0, 4).map((v) => Math.round(v * 1000) / 1000),
          }
        : null;
      this.ws.send(JSON.stringify({ type: "control", data: this.buildControl(), gp }));
    }
    this.publish();

    this.sendTimer = setTimeout(this.sendLoop, 1000 / SEND_HZ);
  };

  // -------------------------------------------------------------------
  // Gamepad.
  //
  // Two browser facts shape this code:
  //   1. navigator.getGamepads() hands back a SNAPSHOT, not a live
  //      object. It must be re-read every frame; a stored reference goes
  //      stale immediately.
  //   2. A pad is invisible to the page until a button on it is pressed.
  //      "Plugged it in and nothing happened" is normal - press a button.
  // -------------------------------------------------------------------

  /**
   * Capture the sticks' resting position automatically, once per pad
   * connection, as soon as they have been still for GP_AUTOCENTRE_MS.
   *
   * This exists because the manual CENTRE STICKS button writes to
   * localStorage, which is per-browser AND per-origin. Calibrating on the
   * laptop therefore did nothing for the same drone opened on a phone, on
   * a teammate's machine, or even on the same laptop via a different
   * address - the stick drifted again with nothing on screen explaining
   * why. Measuring on connection means every device is correct without
   * anyone needing to know the button exists.
   *
   * Guarded: only while disarmed and before control is taken, only if the
   * sticks look plausibly released (see calibrateSticks for the 0.8
   * reasoning), and arming is separately gated on the sticks actually
   * reading zero - so a centre captured while somebody happened to be
   * holding a stick cannot silently become a permanent bias.
   */
  private maybeAutoCentre(rawAxes: number[]) {
    // Blocked only while ARMED. It deliberately does NOT check hasControl:
    // gating on that meant pressing TAKE CONTROL before the one-second
    // window elapsed blocked centring forever, so whether the drift got
    // corrected depended on how fast you clicked. Holding control while
    // disarmed is a perfectly safe moment to measure - throttle is forced
    // to 0 and the motors cannot spin.
    if (this.gpAutoCentred || this.armed) return;
    if (rawAxes.length < 4) return;

    // Rolling window of raw samples. Averaging beats sampling one instant:
    // a worn stick's reading is noisy, so a single frame would bake that
    // frame's noise straight into the centre.
    const win = this.gpSteadyWindow;
    win.push(rawAxes.slice(0, 4));
    if (win.length > GP_AUTOCENTRE_SAMPLES) win.shift();
    if (win.length < GP_AUTOCENTRE_SAMPLES) return;

    // Every axis must have stayed inside the tolerance across the WHOLE
    // window - measured as spread, not as a frame-to-frame delta, so slow
    // creep is caught as well as jitter.
    for (let a = 0; a < 4; a++) {
      const col = win.map((s) => s[a]);
      if (Math.max(...col) - Math.min(...col) > GP_AUTOCENTRE_TOLERANCE) return;
    }

    // Reject a slow sweep: compare the first third of the window with the
    // last third. A stick at rest has no trend; one being moved does.
    const third = Math.floor(win.length / 3);
    for (let a = 0; a < 4; a++) {
      const head = medianOf(win.slice(0, third).map((s) => s[a]));
      const tail = medianOf(win.slice(-third).map((s) => s[a]));
      if (Math.abs(tail - head) > GP_AUTOCENTRE_MAX_TREND) return;
    }

    const centre = [0, 1, 2, 3].map((a) => medianOf(win.map((s) => s[a])));
    // Beyond this it is a held stick, not a worn one - see calibrateSticks.
    if (centre.some((v) => Math.abs(v) > GP_AUTOCENTRE_MAX_OFFSET)) return;

    this.gpAutoCentred = true;
    this.gpCentre = centre;
    try {
      window.localStorage.setItem("stickCentre", JSON.stringify(centre));
    } catch {
      /* private mode - it just will not persist between reloads */
    }
    this.lastEvent = "sticks centred automatically";
  }

  private pollGamepad = () => {
    this.rafId = requestAnimationFrame(this.pollGamepad);

    const pads = navigator.getGamepads ? navigator.getGamepads() : [];
    let pad: Gamepad | null = this.gpIndex !== null ? pads[this.gpIndex] ?? null : null;
    if (!pad || !pad.connected) {
      pad = Array.prototype.find.call(pads, (p: Gamepad | null) => p && p.connected) ?? null;
    }

    if (!pad) {
      if (this.gpIndex !== null) {
        // Poll noticed it vanished even if the event did not fire.
        this.disarm("controller disconnected");
        this.gpIndex = null;
        this.gpInfo = null;
        this.gpAutoCentred = false;
        this.gpSteadyWindow = [];
        this.gpAxisMin = [];
        this.gpAxisMax = [];
    this.gpAxisMin = [];
    this.gpAxisMax = [];
        this.publish();
      }
      return;
    }

    this.gpIndex = pad.index;

    // --- sticks -----------------------------------------------------
    const rawAxes = Array.from(pad.axes);
    this.gpRawAxes = rawAxes;
    for (let a = 0; a < rawAxes.length; a++) {
      this.gpAxisMin[a] = Math.min(this.gpAxisMin[a] ?? rawAxes[a], rawAxes[a]);
      this.gpAxisMax[a] = Math.max(this.gpAxisMax[a] ?? rawAxes[a], rawAxes[a]);
    }
    this.maybeAutoCentre(rawAxes);
    // Re-map each axis around its measured resting point BEFORE the
    // deadzone or anything else sees it. See normaliseAxis(): this is a
    // per-side rescale, not a flat subtraction.
    // normalise (centre + per-side travel) -> jitter filter -> deadzone.
    // Filtering here, before the deadzone, means the deadzone sees an
    // already-clean signal and can therefore be much narrower.
    const ax = (i: number) => {
      const n = normaliseAxis(rawAxes[i] ?? 0, this.gpCentre[i] ?? 0);
      this.gpFiltered[i] = jitterFilter(this.gpFiltered[i] ?? 0, n);
      return this.gpFiltered[i];
    };
    // The browser reports +1 for DOWN on both Y axes; the rest of this
    // file uses +1 for UP, as the on-screen sticks do. Flip once, here.
    const left = applyDeadzone(ax(GP_AXIS_LEFT_X), -ax(GP_AXIS_LEFT_Y));
    const right = applyDeadzone(ax(GP_AXIS_RIGHT_X), -ax(GP_AXIS_RIGHT_Y));

    // Same axis lock the on-screen left stick uses, for the same reason
    // (see LOCK_AT). Re-arms when the stick springs back to centre.
    if (left.mag === 0) this.gpLockedAxis = null;
    else if (this.gpLockedAxis === null && left.mag > LOCK_AT) {
      this.gpLockedAxis = Math.abs(left.x) > Math.abs(left.y) ? "x" : "y";
    }
    if (this.gpLockedAxis === "x") left.y = 0;
    else if (this.gpLockedAxis === "y") left.x = 0;

    this.left = { x: left.x, y: left.y };
    this.right = { x: right.x, y: right.y };

    // --- buttons (edge-triggered: act on the press, not every frame) --
    const pressed = (i: number) => {
      const b = pad!.buttons[i];
      return !!(b && b.pressed);
    };
    const justPressed = (i: number) => pressed(i) && !this.gpPrevButtons[i];

    if (justPressed(GP_BTN_DISARM_A) || justPressed(GP_BTN_DISARM_B)) {
      this.disarm("controller disarm");
    }

    if (pressed(GP_BTN_ARM) && !this.armed) {
      if (!this.gpArmHeldSince) {
        this.gpArmHeldSince = performance.now();
        this.gpArmFired = false;
      } else if (!this.gpArmFired && performance.now() - this.gpArmHeldSince >= GP_ARM_HOLD_MS) {
        // Fire once per press-and-hold, so a refused arm does not retry
        // and flash an error thirty times a second.
        this.gpArmFired = true;
        this.tryArm();
      }
    } else {
      this.gpArmHeldSince = 0;
    }

    // Trim: face buttons, with the D-pad doing the same thing.
    if (justPressed(GP_BTN_TRIM_PITCH_FWD) || justPressed(GP_BTN_DPAD_UP)) {
      this.adjustTrim("pitch", 1);
    }
    if (justPressed(GP_BTN_TRIM_PITCH_BACK) || justPressed(GP_BTN_DPAD_DOWN)) {
      this.adjustTrim("pitch", -1);
    }
    if (justPressed(GP_BTN_TRIM_ROLL_LEFT) || justPressed(GP_BTN_DPAD_LEFT)) {
      this.adjustTrim("roll", -1);
    }
    if (justPressed(GP_BTN_TRIM_ROLL_RIGHT) || justPressed(GP_BTN_DPAD_RIGHT)) {
      this.adjustTrim("roll", 1);
    }

    this.gpPrevButtons = pad.buttons.map((b) => b.pressed);
    let lastButton: number | null = null;
    for (let i = 0; i < this.gpPrevButtons.length; i++) {
      if (this.gpPrevButtons[i]) {
        lastButton = i;
        break;
      }
    }

    const armHoldProgress = this.gpArmHeldSince
      ? Math.min(1, (performance.now() - this.gpArmHeldSince) / GP_ARM_HOLD_MS)
      : 0;

    this.gpInfo = {
      id: pad.id,
      standard: pad.mapping === "standard",
      lx: q(left.x),
      ly: q(left.y),
      rx: q(right.x),
      ry: q(right.y),
      rawAxes: rawAxes.map(q),
      centre: this.gpCentre.map(q),
      rangeMin: this.gpAxisMin.map(q),
      rangeMax: this.gpAxisMax.map(q),
      centred: this.gpCentre.some((v) => v !== 0),
      lastButton,
      armHoldProgress: q(armHoldProgress),
    };

    this.publish();
  };
}

/**
 * Radial deadzone with rescaling: inside the zone the stick reads exactly
 * zero, and travel just outside it starts from zero rather than jumping
 * straight to GP_DEADZONE.
 */
/**
 * Map one raw axis onto a full -1..+1 around its resting point, scaling
 * each side by ITS OWN remaining travel.
 *
 * The obvious approach - subtract the resting offset - is wrong, and was
 * the bug here. Subtracting throws away exactly as much travel as the
 * offset: this pad rests at -0.55 on roll, so `raw - centre` left only
 * 0.45 of range to the left against 1.55 to the right. A full sweep left
 * reached about a third of deflection and felt like it did nothing, while
 * right was over-sensitive. Scaling per side instead means a stick that
 * rests well off-centre still reaches full deflection both ways.
 *
 * Extremes are taken as the spec'd +/-1 rather than the measured range,
 * because a measured maximum is only correct once someone has actually
 * swept the stick that far - and using a half-swept maximum would
 * silently over-scale the control. The divisor floor keeps a stick
 * resting very near an extreme from turning into a huge noise gain
 * (calibration already refuses beyond 0.8, so this is a backstop).
 */
function normaliseAxis(raw: number, centre: number): number {
  const d = raw - centre;
  if (d === 0) return 0;
  const span = d > 0 ? 1 - centre : centre + 1;
  return clamp(d / Math.max(span, 0.2), -1, 1);
}

/**
 * One axis of the adaptive jitter filter. See JITTER_MIN_ALPHA.
 * Quadratic so that the smoothing falls away sharply once movement is
 * clearly deliberate, rather than bleeding lag into medium-speed input.
 */
function jitterFilter(prev: number, next: number): number {
  const d = Math.abs(next - prev);
  const ramp = Math.min(1, d / JITTER_FULL_AT);
  const alpha = JITTER_MIN_ALPHA + (1 - JITTER_MIN_ALPHA) * ramp * ramp;
  return prev + alpha * (next - prev);
}

function applyDeadzone(x: number, y: number) {
  const mag = Math.hypot(x, y);
  if (mag < GP_DEADZONE) return { x: 0, y: 0, mag: 0 };
  const scaled = Math.min(1, (mag - GP_DEADZONE) / (1 - GP_DEADZONE));
  return { x: (x / mag) * scaled, y: (y / mag) * scaled, mag: scaled };
}

function sameGamepad(a: GamepadInfo | null, b: GamepadInfo | null) {
  if (a === b) return true;
  if (!a || !b) return false;
  return (
    a.id === b.id &&
    a.lx === b.lx &&
    a.ly === b.ly &&
    a.rx === b.rx &&
    a.ry === b.ry &&
    a.lastButton === b.lastButton &&
    a.centred === b.centred &&
    a.centre[0] === b.centre[0] &&
    a.centre[1] === b.centre[1] &&
    a.centre[2] === b.centre[2] &&
    a.centre[3] === b.centre[3] &&
    a.armHoldProgress === b.armHoldProgress
  );
}

/** The one link. See the header for why this is a singleton. */
export const droneLink = new DroneLink();

export const TRIM_LIMIT = TRIM_LIMIT_DEG;
export const ARM_HOLD_MS = GP_ARM_HOLD_MS;
export const DEADZONE = GP_DEADZONE;

/** Button bindings, exported so the UI describes exactly what the code does. */
export const BINDINGS = {
  arm: GP_BTN_ARM,
  disarm: [GP_BTN_DISARM_A, GP_BTN_DISARM_B] as const,
  trimPitchFwd: GP_BTN_TRIM_PITCH_FWD,
  trimPitchBack: GP_BTN_TRIM_PITCH_BACK,
  trimRollLeft: GP_BTN_TRIM_ROLL_LEFT,
  trimRollRight: GP_BTN_TRIM_ROLL_RIGHT,
} as const;
