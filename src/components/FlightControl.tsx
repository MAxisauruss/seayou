/**
 * Flight control overlay for the Live Feeds page.
 *
 * The actual flying logic is NOT in here - it lives in
 * services/droneLink.ts, which is a faithful port of the drone's own
 * pilot dashboard. This file is only the face of it. Keeping the two
 * apart is deliberate: a rendering mistake should never be able to
 * change what goes on the wire.
 *
 * Control is opt-in. Until someone presses TAKE CONTROL this page sends
 * nothing at all and behaves exactly as the read-only dashboard always
 * did, which is what makes it safe to leave open next to the pilot
 * dashboard.
 */
import { useState } from "react";
import {
  droneLink,
  useDroneLink,
  LEVEL_RESULT_TEXT,
  ARM_HOLD_MS,
  BINDINGS,
  GP_BUTTON_NAMES,
  TRIM_LIMIT,
} from "../services/droneTelemetry";
import type { DroneBattery, DroneRail, DroneBatteryGuard } from "../services/droneLink";

const btnName = (i: number) => GP_BUTTON_NAMES[i] ?? `button ${i}`;

function StickDot({ x, y, label }: { x: number; y: number; label: string }) {
  return (
    <div className="flex flex-col items-center gap-1">
      <div className="relative w-12 h-12 rounded-full border border-white/15 bg-black/50">
        <div className="absolute left-1/2 top-0 bottom-0 w-px bg-white/10" />
        <div className="absolute top-1/2 left-0 right-0 h-px bg-white/10" />
        <div
          className="absolute w-2.5 h-2.5 rounded-full bg-[#FF6B35] shadow-[0_0_6px_rgba(255,107,53,0.8)]"
          style={{
            left: `calc(50% + ${x * 38}% - 5px)`,
            top: `calc(50% - ${y * 38}% - 5px)`,
          }}
        />
      </div>
      <span className="font-label-caps text-[8px] text-on-surface-variant">{label}</span>
    </div>
  );
}

/**
 * What the on-board low-pack failsafe has decided.
 *
 * This is a REPORT, not a control. By the time "returning" or "landing"
 * shows, the drone has already started doing it on its own and does not
 * need the dashboard - so this is loud, and it says what is happening
 * rather than offering a choice that would not be honoured.
 *
 * WARN is the level that exists for the pilot: nothing automatic has
 * happened yet, and there is still time to land where they want to.
 */
function BatteryGuardBanner({ guard }: { guard: DroneBatteryGuard | null }) {
  if (!guard || guard.level === "ok" || guard.level === "unknown") return null;

  const TEXT: Record<string, { head: string; sub: string; cls: string }> = {
    warn: {
      head: "BATTERY LOW",
      sub: "land soon — nothing automatic yet",
      cls: "bg-[#FFB300]/15 border-[#FFB300] text-[#FFB300]",
    },
    return: {
      head: "RETURNING HOME",
      sub: "low pack — the drone is flying itself back",
      cls: "bg-[#FF6B35]/20 border-[#FF6B35] text-[#FF6B35] animate-pulse",
    },
    land_now: {
      head: "LANDING NOW",
      sub: "critical pack — coming down where it is",
      cls: "bg-[#FF3B30]/20 border-[#FF3B30] text-[#FF6B6B] animate-pulse",
    },
  };
  const t = TEXT[guard.level];
  if (!t) return null;

  return (
    <div
      className={`px-3 py-2 rounded border font-label-caps ${t.cls}`}
      title={guard.reason || t.sub}
    >
      <span className="text-label-caps tracking-wider">{t.head}</span>
      <span className="block font-telemetry-sm text-[10px] opacity-90">
        {guard.cell_v != null
          ? `${guard.cell_v.toFixed(2)} V/cell · ${t.sub}`
          : t.sub}
      </span>
    </div>
  );
}

/**
 * Pack voltage, shown under SET LEVEL.
 *
 * Colour is by volts PER CELL, not pack volts, because 11.1 V is a healthy
 * 3S and a dangerously flat 4S. The cell count comes from the Pi, which
 * infers it from the voltage; without it this shows the pack voltage plainly
 * and stays neutral rather than colouring it on a guess.
 *
 *   >= 3.8 V/cell  green    healthy
 *   >= 3.5 V/cell  amber    getting low, land soon
 *   <  3.5 V/cell  red      land now
 *
 * A fault is rendered AS a fault - never as 0.0 V, which would read as a
 * flat pack and is the one thing this must not imply.
 */
function BatteryReadout({
  batt,
  rail,
}: {
  batt: DroneBattery | null;
  rail: DroneRail | null;
}) {
  if (!batt && !rail) return null;

  // Kept short enough for the flight bar; the full explanation is the
  // tooltip. "pack across shunt" is the dangerous one - the battery is wired
  // straight across VIN+/VIN-, which is the 0.1 ohm shunt across the pack.
  const FAULT_TEXT: Record<string, string> = {
    pack_across_shunt: "pack wired across VIN+/VIN−",
    no_pack: "no pack on VIN−",
    open_shunt: "VIN− not on the pack",
    overflow: "sensor overflow",
    i2c_error: "no I²C reply",
  };

  const packUnusable =
    !batt || batt.fault || batt.voltage_v === null || batt.voltage_v === undefined;

  const perCell = batt?.cell_v ?? null;
  const packColour =
    perCell === null
      ? "text-on-surface"
      : perCell >= 3.8
        ? "text-[#00E676]"
        : perCell >= 3.5
          ? "text-[#FFB300]"
          : "text-[#FF3B30]";

  return (
    <div className="flex flex-col leading-tight gap-0.5">
      {/* ---- pack voltage, from the INA219 ---- */}
      <span className="font-label-caps text-[8px] text-on-surface-variant">BATTERY</span>
      {packUnusable ? (
        <span
          className="font-telemetry-sm text-[11px] text-[#FFB300]"
          title={
            batt?.fault === "pack_across_shunt"
              ? "The battery is wired across VIN+ and VIN−, which puts the pack straight across the module's 0.1 ohm shunt. The chip measures bus voltage AT VIN−, and here VIN− is battery negative, so it reads 0 V and always will. Disconnect the pack and move battery− to the GND pin before changing anything else."
              : batt?.fault === "open_shunt"
                ? "The INA219 is powered and talking, but its VIN− terminal is not on the pack, so it cannot measure anything. It is not reporting 0 V, because that would look like a flat battery."
                : "No trustworthy pack reading."
          }
        >
          NO READING
          <span className="block font-label-caps text-[7px] text-on-surface-variant">
            {batt ? (FAULT_TEXT[batt.fault ?? ""] ?? "unavailable") : "no sensor"}
          </span>
        </span>
      ) : (
        <span
          className={`font-telemetry-sm text-[13px] font-bold ${packColour}`}
          title={`${batt!.voltage_v!.toFixed(2)} V`}
        >
          {batt!.voltage_v!.toFixed(2)} V
          <span className="block font-label-caps text-[7px] text-on-surface-variant">
            {batt!.cells ? `${batt!.cells}S · ${perCell?.toFixed(2)} V/cell` : "pack"}
            {batt!.current_a != null ? ` · ${batt!.current_a.toFixed(1)} A` : ""}
          </span>
        </span>
      )}

      {/* ---- 5 V rail, from the Pico's own ADC ----
          Always shown, because it needs no external sensor and so it works
          when the INA219 cannot. It is also the rail the GPS and the I2C
          sensors run from, so "usb only" here is a direct explanation for a
          GPS that never transmits. */}
      {rail && (
        <span
          className="font-label-caps text-[8px] text-on-surface-variant pt-0.5 border-t border-white/10"
          title={
            rail.source === "usb_backfeed"
              ? "Around 4.8 V is USB power leaking through the Pico's Schottky diode. The ESC BEC is NOT supplying this rail - and the GPS and I2C sensors run from it."
              : "The ESC BEC is supplying the 5 V rail."
          }
        >
          5V RAIL{" "}
          <span
            className={`font-telemetry-sm text-[10px] ${
              rail.vsys_v < 4.4
                ? "text-[#FF3B30]"
                : rail.source === "esc_bec"
                  ? "text-[#00E676]"
                  : "text-[#FFB300]"
            }`}
          >
            {rail.vsys_v.toFixed(2)} V
          </span>
          <span className="block font-label-caps text-[7px] text-on-surface-variant">
            {rail.source === "esc_bec" ? "ESC BEC live" : "USB only — BEC off"}
          </span>
        </span>
      )}
    </div>
  );
}

export default function FlightControl() {
  const s = useDroneLink();
  const [showMapping, setShowMapping] = useState(false);

  const gp = s.gamepad;
  const tele = s.telemetry;
  const motors = tele?.motor_out_debug;
  const spread = motors && motors.length === 4 ? Math.max(...motors) - Math.min(...motors) : null;
  // The rate loop runs off the raw gyro, so a rejected bias sweep means no
  // bias correction at all and a steady drift to one side. Loud, not subtle.
  const gyroBad = tele?.gyro_cal_ok === false;

  // The stored level zero-point, null on firmware that predates it.
  const lvl = tele?.level ?? null;
  // Pack voltage from the INA219 on the Pi. Absent when none is fitted.
  const batt = tele?.battery ?? null;
  // The 5 V rail, straight off the Pico's ADC - works with no external sensor.
  const rail = tele?.rail ?? null;
  // What the drone's own low-pack failsafe has decided. Report, not control.
  const guard = tele?.battery_guard ?? null;

  const armLabel = btnName(BINDINGS.arm);
  const disarmLabel = BINDINGS.disarm.map(btnName).join(" or ");
  // Post-deadzone values are exactly zero when the stick is genuinely at
  // rest, so anything non-zero here is real deflection reaching the wire.
  // Worth saying out loud while disarmed: on this pad the right stick sat
  // at -0.55 untouched, which would have rolled the drone on arming.
  const sticksOffCentre =
    !!gp && !s.armed && (gp.lx !== 0 || gp.ly !== 0 || gp.rx !== 0 || gp.ry !== 0);

  // No positioning of its own any more. It used to pin itself over the
  // bottom of the video with `absolute inset` and pointer-events juggling;
  // it now sits in the control column and the parent decides where that
  // goes. Nothing it renders can cover the camera.
  return (
    <div>
      <div className="glass-panel rounded-lg">
        {/* ---- main bar ---- */}
        <div className="flex items-center gap-4 p-3 flex-wrap">
          {/* Arm state */}
          <div
            className={`px-3 py-2 rounded border font-label-caps text-label-caps tracking-wider ${
              s.armed
                ? "bg-[#FF3B30]/15 border-[#FF3B30] text-[#FF6B6B] animate-pulse"
                : "bg-white/5 border-white/15 text-on-surface-variant"
            }`}
          >
            {s.armed ? "ARMED" : "DISARMED"}
          </div>

          <BatteryGuardBanner guard={guard} />

          {/* Take control gate.
              Arming is deliberately two deliberate steps, but a greyed-out
              ARM button with no stated reason just reads as broken - so
              while control is released the two are numbered, and the one
              you need next is the one that is lit. */}
          <div className="flex flex-col gap-0.5">
            {!s.hasControl && (
              <span className="font-label-caps text-[8px] text-[#FF6B35]">STEP 1</span>
            )}
            <button
              onClick={() => droneLink.takeControl(!s.hasControl)}
              title={
                s.hasControl
                  ? "Stop sending commands to the drone. Disarms first."
                  : "This page sends nothing to the drone until you press this."
              }
              className={`px-4 py-2 rounded font-label-caps text-label-caps border transition-colors ${
                s.hasControl
                  ? "bg-[#FF6B35]/20 border-[#FF6B35] text-[#FF6B35] hover:bg-[#FF6B35]/30"
                  : "bg-[#FF6B35]/15 border-[#FF6B35] text-[#FF6B35] hover:bg-[#FF6B35]/25 animate-pulse"
              }`}
            >
              {s.hasControl ? "RELEASE CONTROL" : "TAKE CONTROL"}
            </button>
          </div>

          {/* Arm / disarm */}
          <div className="flex flex-col gap-0.5">
            {!s.hasControl && (
              <span className="font-label-caps text-[8px] text-on-surface-variant">
                STEP 2 — LOCKED
              </span>
            )}
            <button
              onClick={() => droneLink.toggleArm()}
              disabled={!s.hasControl}
              title={
                !s.hasControl
                  ? "Press TAKE CONTROL first — this page is read-only until then."
                  : s.armed
                    ? "Stop the motors."
                    : "Arm the motors. Sticks must be centred and throttle at zero."
              }
              className={`px-5 py-2 rounded font-bold tracking-wide transition-all active:scale-95 disabled:opacity-30 disabled:cursor-not-allowed ${
                s.armed
                  ? "bg-[#FF3B30] text-white shadow-[0_0_18px_rgba(255,59,48,0.5)]"
                  : "bg-[#00E676] text-[#04251b]"
              }`}
            >
              {s.armed ? "DISARM" : "ARM"}
            </button>
          </div>

          {/* Throttle */}
          <div className="flex flex-col gap-1 min-w-[140px] flex-1">
            <div className="flex justify-between">
              <span className="font-label-caps text-[9px] text-on-surface-variant">THROTTLE</span>
              <span
                className={`font-telemetry-sm text-telemetry-sm ${
                  s.throttle > 0 ? "text-[#FF6B35]" : "text-white"
                }`}
              >
                {s.throttle}%
              </span>
            </div>
            <div className="h-2 rounded bg-black/60 border border-white/10 overflow-hidden">
              <div
                className="h-full bg-gradient-to-r from-[#f09f33] to-[#FF6B35] transition-[width] duration-75"
                style={{ width: `${s.throttle}%` }}
              />
            </div>
          </div>

          {/* Live stick positions */}
          {gp && (
            <div className="flex gap-3">
              <StickDot x={gp.lx} y={gp.ly} label="YAW / THR" />
              <StickDot x={gp.rx} y={gp.ry} label="ROLL / PITCH" />
            </div>
          )}

          {/* What is ACTUALLY going on the wire. 50 is neutral.
              This is here because the raw axis numbers in the panel below
              jitter constantly on a worn stick even when the output is a
              clean 50 - so watching those makes a healthy setup look
              broken. If these three read 50, the drone is being told to
              hold level, whatever the raw numbers are doing. */}
          <div className="flex flex-col gap-0.5 px-3 py-1.5 rounded bg-black/40 border border-white/10">
            <span className="font-label-caps text-[8px] text-on-surface-variant">
              COMMANDED (50 = NEUTRAL)
            </span>
            <div className="flex gap-2 font-telemetry-sm text-[11px]">
              {(
                [
                  ["R", s.command.roll],
                  ["P", s.command.pitch],
                  ["Y", s.command.yaw],
                ] as const
              ).map(([k, v]) => (
                <span key={k} className={v === 50 ? "text-[#00E676]" : "text-[#FF6B35]"}>
                  {k} {v}
                </span>
              ))}
            </div>
          </div>

          {/* Controller status */}
          <button
            onClick={() => setShowMapping((v) => !v)}
            className="flex items-center gap-2 px-3 py-2 rounded border border-white/15 bg-black/30 hover:bg-white/5"
          >
            <span
              className={`w-2 h-2 rounded-full ${gp ? "bg-[#00E676]" : "bg-white/25"}`}
            />
            <span className="font-label-caps text-[10px] text-on-surface-variant max-w-[150px] truncate">
              {gp ? gp.id.replace(/\s*\(.*\)\s*/, "") || "CONTROLLER" : "NO CONTROLLER"}
            </span>
            <span className="material-symbols-outlined text-sm text-on-surface-variant">
              {showMapping ? "expand_more" : "expand_less"}
            </span>
          </button>

          {/* SET LEVEL - tell the aircraft what flat is.
              Here, in the always-visible bar, rather than in the AUTONOMOUS
              panel: that panel only renders when the ground station reports
              a mission block, and it sits in a scrolling box. This is a
              pre-flight action wanted before every session, so it lives
              where TAKE CONTROL and ARM live.

              Hidden only on firmware with no level block at all, rather
              than showing a button that cannot do anything. */}
          {/* SET LEVEL and, directly beneath it, the pack voltage. They share
              one column so the voltage sits under the button rather than
              beside it. The column renders whenever EITHER is available, so
              the voltage still shows on firmware with no level block. */}
          {(lvl || batt || rail) && (
            <div className="flex flex-col gap-0.5">
              {lvl && (
                <>
                  <span className="font-label-caps text-[8px] text-on-surface-variant">
                    LEVEL {lvl.roll_offset_deg.toFixed(1)}° / {lvl.pitch_offset_deg.toFixed(1)}°
                  </span>
                  <button
                    onClick={() => droneLink.requestLevelCal("start")}
                    disabled={lvl.busy || s.armed}
                    title="Flat surface, throttle at zero, hands off for three seconds."
                    className="px-3 py-2 rounded bg-[#00E676] text-[#04251b] font-bold
                               text-[11px] tracking-wide active:scale-95 transition-transform
                               disabled:opacity-30 disabled:cursor-not-allowed"
                  >
                    {lvl.state === "settling"
                      ? "SETTLING…"
                      : lvl.state === "sampling"
                        ? "MEASURING…"
                        : "SET LEVEL"}
                  </button>
                </>
              )}
              <BatteryReadout batt={batt} rail={rail} />
            </div>
          )}

          {/* Emergency stop - always live, even without control taken, so it
              is never the button you have to think about. */}
          <button
            onClick={() => droneLink.disarm("emergency stop")}
            className="px-4 py-2 rounded bg-[#FF3B30] text-white font-bold tracking-wide active:scale-95 transition-transform"
          >
            STOP
          </button>
        </div>

        {/* ---- arm-hold progress ---- */}
        {gp && gp.armHoldProgress > 0 && !s.armed && (
          <div className="px-3 pb-2">
            <div className="flex justify-between mb-1">
              <span className="font-label-caps text-[9px] text-tertiary">
                HOLD {armLabel.toUpperCase()} TO ARM
              </span>
              <span className="font-telemetry-sm text-[10px] text-tertiary">
                {Math.round(gp.armHoldProgress * ARM_HOLD_MS)}ms
              </span>
            </div>
            <div className="h-1 rounded bg-black/60 overflow-hidden">
              <div
                className="h-full bg-[#00E676]"
                style={{ width: `${gp.armHoldProgress * 100}%` }}
              />
            </div>
          </div>
        )}

        {/* ---- status line ---- */}
        <div className="px-3 pb-2 flex items-center gap-3 flex-wrap">
          {(s.levelMessage || lvl?.result) && (
            <span
              className={`font-label-caps text-[9px] ${
                lvl?.result && lvl.result !== "ok"
                  ? "text-[#FF3B30]"
                  : "text-[#00E676]"
              }`}
            >
              LEVEL: {s.levelMessage || LEVEL_RESULT_TEXT[lvl?.result ?? ""]}
            </span>
          )}
          {!s.hasControl && (
            <span className="font-label-caps text-[9px] text-on-surface-variant">
              READ-ONLY // PRESS TAKE CONTROL TO FLY — CLOSE THE PILOT DASHBOARD FIRST
            </span>
          )}
          {s.hasControl && (
            <span className="font-label-caps text-[9px] text-[#FF6B35]">
              THIS PAGE IS FLYING THE DRONE // {disarmLabel.toUpperCase()} OR SPACEBAR STOPS IT
            </span>
          )}
          {s.lastEvent && (
            <span className="font-telemetry-sm text-[10px] text-tertiary">· {s.lastEvent}</span>
          )}
          {gyroBad && (
            <span className="font-label-caps text-[9px] text-[#FF3B30]">
              ⚠ GYRO CAL FAILED — POWER-CYCLE THE PICO STANDING STILL, DO NOT FLY
            </span>
          )}
          {sticksOffCentre && (
            <span className="font-label-caps text-[9px] text-[#FF6B35]">
              ⚠ STICKS NOT AT ZERO — LET GO, THEN PRESS CENTRE STICKS
            </span>
          )}
          {spread !== null && (
            <span
              className={`font-telemetry-sm text-[10px] ${
                spread > 80 ? "text-[#FF6B35]" : "text-on-surface-variant"
              }`}
            >
              MOTOR SPREAD {spread}
            </span>
          )}
        </div>

        {/* ---- controller mapping / trim (collapsible) ---- */}
        {showMapping && (
          <div className="border-t border-white/10 p-3 flex flex-col gap-3">
            {!gp && (
              <p className="font-body-md text-xs text-on-surface-variant">
                No controller seen yet. Plug the pad in (or pair it over Bluetooth), then{" "}
                <b className="text-white">press any button on it</b> — browsers deliberately hide a
                gamepad from the page until it is used at least once. Click this page first so it
                has keyboard focus.
              </p>
            )}

            {gp && (
              <>
                <div className="flex flex-wrap gap-x-6 gap-y-2">
                  <div className="flex flex-col">
                    <span className="font-label-caps text-[9px] text-on-surface-variant">PAD</span>
                    <span className="font-telemetry-sm text-[11px] text-white">{gp.id}</span>
                  </div>
                  <div className="flex flex-col">
                    <span className="font-label-caps text-[9px] text-on-surface-variant">
                      MAPPING
                    </span>
                    <span
                      className={`font-telemetry-sm text-[11px] ${
                        gp.standard ? "text-[#00E676]" : "text-[#FF6B35]"
                      }`}
                    >
                      {gp.standard ? "STANDARD" : "NON-STANDARD"}
                    </span>
                  </div>
                  <div className="flex flex-col">
                    <span className="font-label-caps text-[9px] text-on-surface-variant">
                      BUTTON PRESSED
                    </span>
                    <span className="font-telemetry-sm text-[11px] text-white">
                      {gp.lastButton === null
                        ? "—"
                        : `#${gp.lastButton}${
                            gp.standard && GP_BUTTON_NAMES[gp.lastButton]
                              ? ` · ${GP_BUTTON_NAMES[gp.lastButton]}`
                              : ""
                          }`}
                    </span>
                  </div>
                </div>

                <div className="flex flex-col gap-1">
                  <span className="font-label-caps text-[9px] text-on-surface-variant">
                    RAW AXES (0=YAW 1=THROTTLE 2=ROLL 3=PITCH)
                  </span>
                  <div className="flex flex-wrap gap-2">
                    {gp.rawAxes.map((v, i) => {
                      const corrected = v - (gp.centre[i] ?? 0);
                      const off = Math.abs(corrected) > 0.15;
                      return (
                        <span
                          key={i}
                          className={`font-telemetry-sm text-[10px] px-2 py-0.5 rounded bg-black/40 border ${
                            off ? "border-[#FF6B35] text-[#FF6B35]" : "border-white/10 text-white"
                          }`}
                          title={`raw ${v.toFixed(3)} − centre ${(gp.centre[i] ?? 0).toFixed(3)}`}
                        >
                          {i}: {corrected.toFixed(2)}
                        </span>
                      );
                    })}
                  </div>
                </div>

                {/* Travel check. Sweep each stick fully in both directions and
                    these fill in. A healthy axis reaches about -1.00 .. +1.00.
                    An axis that only reaches, say, -0.60 .. +1.00 has lost
                    travel on one side - that is a hardware fault the software
                    cannot correct, and it is the difference between a stick
                    that merely rests off-centre and one that is failing. */}
                <div className="flex flex-col gap-1">
                  <span className="font-label-caps text-[9px] text-on-surface-variant">
                    TRAVEL — SWEEP BOTH STICKS FULLY; HEALTHY IS −1.00 TO +1.00
                  </span>
                  <div className="flex flex-wrap gap-2">
                    {["YAW", "THR", "ROLL", "PITCH"].map((label, i) => {
                      const lo = gp.rangeMin[i];
                      const hi = gp.rangeMax[i];
                      if (lo === undefined || hi === undefined) return null;
                      // Only judge once the axis has actually been swept.
                      const swept = hi - lo > 0.5;
                      const bad = swept && (lo > -0.85 || hi < 0.85);
                      return (
                        <span
                          key={label}
                          className={`font-telemetry-sm text-[10px] px-2 py-0.5 rounded bg-black/40 border ${
                            bad ? "border-[#FF3B30] text-[#FF3B30]" : "border-white/10 text-white"
                          }`}
                        >
                          {label} {lo.toFixed(2)} … {hi.toFixed(2)}
                          {bad ? " ⚠" : ""}
                        </span>
                      );
                    })}
                  </div>
                </div>

                {/* Stick centring. A worn thumbstick does not spring back to
                    exactly zero, and on the roll axis that means the drone
                    flies sideways the moment it arms. Measuring the resting
                    position beats widening the deadzone, which would throw
                    away most of the stick's usable travel. */}
                <div className="flex items-center gap-3 flex-wrap">
                  <button
                    onClick={() => droneLink.calibrateSticks()}
                    disabled={s.armed}
                    className="px-3 py-1.5 rounded border border-[#00E676]/60 text-[#00E676] hover:bg-[#00E676]/10 font-label-caps text-[10px] disabled:opacity-30 disabled:cursor-not-allowed"
                  >
                    CENTRE STICKS
                  </button>
                  {gp.centred && (
                    <button
                      onClick={() => droneLink.resetStickCentre()}
                      disabled={s.armed}
                      className="px-3 py-1.5 rounded border border-white/15 text-on-surface-variant hover:text-white font-label-caps text-[10px] disabled:opacity-30"
                    >
                      CLEAR
                    </button>
                  )}
                  <span
                    className={`font-telemetry-sm text-[10px] ${
                      gp.centred ? "text-[#00E676]" : "text-on-surface-variant"
                    }`}
                  >
                    {gp.centred
                      ? `centred: ${gp.centre.map((v) => v.toFixed(2)).join(", ")}`
                      : "not centred — take your hands off and press CENTRE STICKS"}
                  </span>
                </div>

                {!gp.standard && (
                  <p className="font-body-md text-xs text-[#FF6B35]">
                    This pad is not using the browser's standard layout, so the axis and button
                    numbers above may not match the defaults. Press each stick and button, note the
                    numbers, and correct the <code>GP_AXIS_*</code> / <code>GP_BTN_*</code> constants
                    at the top of <code>services/droneLink.ts</code>.
                  </p>
                )}

                <div className="font-body-md text-xs text-on-surface-variant flex flex-col gap-1">
                  <p className="m-0">
                    <b className="text-white">Left stick</b> up/down ramps the throttle and{" "}
                    <b className="text-white">holds it</b> · <b className="text-white">left stick</b>{" "}
                    sideways is yaw · <b className="text-white">right stick</b> is roll and pitch.
                  </p>
                  <p className="m-0">
                    <b className="text-white">Hold {armLabel}</b> to arm ·{" "}
                    <b className="text-[#FF6B6B]">{disarmLabel}</b> disarms instantly.
                  </p>
                  <p className="m-0">
                    Trim on the face buttons, laid out the way they sit on the pad:{" "}
                    <b className="text-white">{btnName(BINDINGS.trimPitchFwd)}</b> nose forward ·{" "}
                    <b className="text-white">{btnName(BINDINGS.trimPitchBack)}</b> nose back ·{" "}
                    <b className="text-white">{btnName(BINDINGS.trimRollLeft)}</b> roll left ·{" "}
                    <b className="text-white">{btnName(BINDINGS.trimRollRight)}</b> roll right. The
                    D-pad does the same four.
                  </p>
                  <p className="m-0 text-[#FF6B35]">
                    Releasing the throttle stick does <b>not</b> stop the motors — only disarming
                    does.
                  </p>
                </div>
              </>
            )}

            {/* Trim */}
            <div className="flex items-center gap-4 flex-wrap border-t border-white/10 pt-3">
              <span className="font-label-caps text-[9px] text-on-surface-variant">
                DRIFT TRIM (MAX ±{TRIM_LIMIT}°)
              </span>
              {(["roll", "pitch"] as const).map((axis) => (
                <div key={axis} className="flex items-center gap-1">
                  <span className="font-label-caps text-[9px] text-on-surface-variant w-9">
                    {axis.toUpperCase()}
                  </span>
                  <button
                    onClick={() => droneLink.adjustTrim(axis, -1)}
                    className="w-7 h-7 rounded border border-white/15 bg-black/30 text-white hover:bg-white/10"
                  >
                    −
                  </button>
                  <span className="font-telemetry-sm text-[11px] text-white w-10 text-center">
                    {(axis === "roll" ? s.rollTrim : s.pitchTrim) > 0 ? "+" : ""}
                    {axis === "roll" ? s.rollTrim : s.pitchTrim}°
                  </span>
                  <button
                    onClick={() => droneLink.adjustTrim(axis, 1)}
                    className="w-7 h-7 rounded border border-white/15 bg-black/30 text-white hover:bg-white/10"
                  >
                    +
                  </button>
                </div>
              ))}
              <button
                onClick={() => droneLink.resetTrim()}
                className="px-3 py-1 rounded border border-white/15 text-on-surface-variant hover:text-white font-label-caps text-[9px]"
              >
                RESET TRIM
              </button>
            </div>
          </div>
        )}
      </div>
    </div>
  );
}
