"use strict";

// ---------------------------------------------------------------------
// State
// ---------------------------------------------------------------------

const state = {
  armed: false,
  left: { x: 0, y: 0 },   // yaw, throttle
  right: { x: 0, y: 0 },  // roll, pitch
  ws: null,
  wsConnected: false,
  lastTelemetryAt: 0,
  // Accumulated throttle, 0-100. The left stick does NOT set this directly -
  // it sets how fast it changes. See updateThrottle().
  throttle: 0,
  // Roll/pitch trim in DEGREES. The firmware adds this to its stored level
  // zero-point (each 1 deg = 1 protocol unit; byte 25 = 0 deg). This is the
  // in-flight fix for steady drift: a quad has no position hold, so if it
  // slides to one side you trim the opposite way until it sits still.
  // Persisted so it survives a page reload.
  rollTrim: 0,
  pitchTrim: 0,
  // Sticky message from the last SET LEVEL / ERASE press, shown until the
  // Pico's own telemetry has something newer to say about it.
  levelMsg: "",
  levelMsgAt: 0,
};

const SEND_HZ = 30;

// ---------------------------------------------------------------------
// Stick to protocol-byte mapping.
//
// Throttle is RATE-controlled, not position-controlled. Every stick
// springs back to centre as normal, including this one - but while the
// throttle stick is held off-centre it ramps the throttle up or down at a
// speed proportional to how far it is pushed. Let go, the stick re-centres,
// and the throttle stays wherever it got to. That is what lets it sit at a
// hover instead of dropping the moment you release.
//
// Push up  = climb the throttle.  Push down = wind it back.  Release = hold.
//
// So releasing the stick no longer stops the motors. DISARM is the only
// thing that does: red button, ARM toggle, spacebar, switching tabs, or
// losing the WebSocket - all zero state.throttle and force 0 on the wire.
// Byte protocol: roll/pitch/yaw are 0-100 with 50=center; throttle is
// 0-100 with 0=motors off.

// Percent of full throttle added per second at FULL stick deflection.
// Half deflection ramps at half this rate, so small nudges give fine trim
// near hover while a big push moves quickly.
const THROTTLE_RATE = 25;
// Ignore tiny residual deflection so the throttle never creeps on its own.
const THROTTLE_DEADZONE = 0.05;
//
// Pitch sign convention (stick-up = positive ctrl_pitch on the Pico,
// see Tx_Rx_Update_Variables() in fc_quad_gt_cc.cpp) is a best guess
// made without being able to test against real hardware - verify with
// props OFF before ever arming with props on, and flip PITCH_INVERTED
// below if it's backwards.
// ---------------------------------------------------------------------

const PITCH_INVERTED = false;

// Integrate the throttle stick into the held throttle level. Called from
// the send loop with the real elapsed time, so the ramp rate stays honest
// even if the browser's timers drift.
function updateThrottle(dtSeconds) {
  if (!state.armed) {
    state.throttle = 0;
    return;
  }
  const y = state.left.y;
  if (Math.abs(y) > THROTTLE_DEADZONE) {
    state.throttle += y * THROTTLE_RATE * dtSeconds;
    state.throttle = Math.max(0, Math.min(100, state.throttle));
  }
}

function controlBytes() {
  const yaw = 50 + Math.round(state.left.x * 50);
  const throttle = Math.round(state.throttle);
  const roll = 50 + Math.round(state.right.x * 50);
  const pitchNorm = PITCH_INVERTED ? -state.right.y : state.right.y;
  const pitch = 50 - Math.round(pitchNorm * 50);

  return {
    roll,
    pitch,
    throttle: state.armed ? throttle : 0,
    yaw,
    cmd0: 0,
    cmd1: 0,
    // 25 = zero trim; each unit is 1 deg. Clamped to the protocol byte's
    // safe range so a runaway value can never be sent.
    roll_trim: clampTrimByte(25 + Math.round(state.rollTrim)),
    pitch_trim: clampTrimByte(25 + Math.round(state.pitchTrim)),
  };
}

const TRIM_LIMIT_DEG = 12;   // sensible max; well inside the firmware range

function clampTrimByte(v) {
  return Math.max(25 - TRIM_LIMIT_DEG, Math.min(25 + TRIM_LIMIT_DEG, v));
}

// Nudge a trim axis by delta degrees, update the readout, and remember it.
function adjustTrim(axis, delta) {
  const key = axis === "roll" ? "rollTrim" : "pitchTrim";
  state[key] = Math.max(-TRIM_LIMIT_DEG, Math.min(TRIM_LIMIT_DEG, state[key] + delta));
  try { localStorage.setItem(key, String(state[key])); } catch { /* private mode */ }
  renderTrim();
}

function renderTrim() {
  const r = document.getElementById("roll-trim-val");
  const p = document.getElementById("pitch-trim-val");
  if (r) r.textContent = `${state.rollTrim > 0 ? "+" : ""}${state.rollTrim}°`;
  if (p) p.textContent = `${state.pitchTrim > 0 ? "+" : ""}${state.pitchTrim}°`;
}

// ---------------------------------------------------------------------
// WebSocket
// ---------------------------------------------------------------------

function connectWs() {
  const proto = location.protocol === "https:" ? "wss:" : "ws:";
  const ws = new WebSocket(`${proto}//${location.host}/ws`);
  state.ws = ws;

  ws.onopen = () => {
    state.wsConnected = true;
    setPill("link-pill", "link-text", "ok", "Connected");
  };

  ws.onclose = () => {
    state.wsConnected = false;
    setPill("link-pill", "link-text", "", "Disconnected – retrying…");
    disarm();
    setTimeout(connectWs, 1000);
  };

  ws.onerror = () => ws.close();

  ws.onmessage = (evt) => {
    let msg;
    try { msg = JSON.parse(evt.data); } catch { return; }
    if (msg.type === "telemetry") renderTelemetry(msg.data);
    else if (msg.type === "level_cal_ack") {
      state.levelMsg = msg.data.accepted ? "Capture requested…" : `Refused: ${msg.data.reason}`;
      state.levelMsgAt = performance.now();
    }
  };
}

let lastSendAt = performance.now();

function sendControlLoop() {
  const now = performance.now();
  // Clamped: a backgrounded tab can stall timers for seconds, and that
  // must never arrive as one huge throttle step.
  const dt = Math.min((now - lastSendAt) / 1000, 0.1);
  lastSendAt = now;
  updateThrottle(dt);

  const c = controlBytes();
  if (state.wsConnected && state.ws.readyState === WebSocket.OPEN) {
    state.ws.send(JSON.stringify({ type: "control", data: c }));
  }
  // The throttle now latches, so what is actually being commanded is no
  // longer obvious from where your thumb is. Show it.
  const el = document.getElementById("throttle-val");
  if (el) {
    el.textContent = `${c.throttle}%`;
    el.style.color = c.throttle > 0 ? "var(--warn)" : "";
  }
  // 50 is centre. Anything else while you are only trying to climb is
  // unwanted yaw leaking in - the thing the axis lock exists to stop.
  const yel = document.getElementById("yawcmd-val");
  if (yel) {
    yel.textContent = String(c.yaw);
    yel.style.color = c.yaw !== 50 ? "var(--warn)" : "";
  }
  setTimeout(sendControlLoop, 1000 / SEND_HZ);
}

function setPill(pillId, textId, cls, text) {
  const pill = document.getElementById(pillId);
  pill.classList.remove("ok", "warn");
  if (cls) pill.classList.add(cls);
  document.getElementById(textId).textContent = text;
}

// ---------------------------------------------------------------------
// Telemetry rendering
// ---------------------------------------------------------------------

function renderTelemetry(data) {
  state.lastTelemetryAt = performance.now();

  const { roll, pitch, yaw } = data.attitude;
  document.getElementById("roll-val").textContent = `${roll.toFixed(1)}°`;
  document.getElementById("pitch-val").textContent = `${pitch.toFixed(1)}°`;
  document.getElementById("yaw-val").textContent = `${yaw.toFixed(1)}°`;

  const tilt = document.getElementById("horizon-tilt");
  const clampedPitch = Math.max(-45, Math.min(45, pitch));
  tilt.setAttribute("transform", `rotate(${-roll} 100 100) translate(0 ${clampedPitch * 1.6})`);

  const gps = data.gps;
  document.getElementById("fix-val").textContent = gps.fix ? `Type ${gps.fix}` : "No fix";
  document.getElementById("sat-val").textContent = gps.sat_count;
  setPill("gps-pill", "gps-text", gps.has_fix ? "ok" : "warn",
    gps.has_fix ? `GPS: ${gps.sat_count} sats` : "GPS: no fix");

  const baro = data.baro;
  document.getElementById("baro-pressure-val").textContent =
    baro ? `${baro.pressure_hpa.toFixed(1)} hPa` : "No sensor";
  document.getElementById("baro-temp-val").textContent =
    baro ? `${baro.temp_c.toFixed(0)}°C` : "—";
  document.getElementById("baro-height-val").textContent =
    baro ? `${baro.height_m.toFixed(1)} m` : "—";

  // Per-motor outputs the Pico actually computed. Worth showing: if the
  // motors are uneven, this says whether it is coming from an unwanted yaw
  // (motors 0+2 differ from 1+3) or from the levelling PIDs correcting a
  // tilt, which is normal and expected whenever the drone is not level.
  const m = data.motor_out_debug;
  if (m && m.length === 4) {
    document.getElementById("motors-val").textContent = m.join(" · ");
    const spread = Math.max(...m) - Math.min(...m);
    const el = document.getElementById("motorspread-val");
    el.textContent = String(spread);
    el.style.color = spread > 80 ? "var(--warn)" : "";
  }

  // The rate loop runs off the raw gyro now, so a rejected bias sweep means
  // no bias correction at all and a steady drift to one side. Make that
  // loud rather than something you discover in the air.
  const gc = document.getElementById("gyrocal-val");
  if (gc) {
    if (data.gyro_cal_ok === true) {
      gc.textContent = "OK";
      gc.style.color = "var(--good)";
    } else if (data.gyro_cal_ok === false) {
      gc.textContent = "FAILED – power-cycle still";
      gc.style.color = "var(--bad)";
    } else {
      gc.textContent = "—";
      gc.style.color = "";
    }
  }

  renderLevel(data.level);
}

// ---------------------------------------------------------------------
// Level zero-point panel.
//
// data.level is null on Pico firmware built before Level_Capture_Run()
// existed - hide the whole panel in that case rather than showing 0.00°
// that nothing is actually reporting.
// ---------------------------------------------------------------------

const LEVEL_RESULT_TEXT = {
  ok: "Captured and saved",
  moved: "Aborted – the drone moved",
  out_of_range: "Aborted – more than 15° out, check the mounting",
  throttle_not_zero: "Aborted – throttle must be at zero",
};

function renderLevel(level) {
  const block = document.getElementById("level-block");
  if (!level) {
    block.hidden = true;
    return;
  }
  block.hidden = false;

  document.getElementById("level-offset-val").textContent =
    `${level.roll_offset_deg.toFixed(2)}° / ${level.pitch_offset_deg.toFixed(2)}°`;

  let status;
  if (level.state === "settling") status = "Settling…";
  else if (level.state === "sampling") status = "Measuring…";
  else if (state.levelMsg && performance.now() - state.levelMsgAt < 4000) status = state.levelMsg;
  else status = LEVEL_RESULT_TEXT[level.result] || "Idle";

  document.getElementById("level-status-val").textContent = status;
  document.getElementById("level-btn").disabled = level.busy || state.armed;
  document.getElementById("level-erase-btn").disabled = level.busy || state.armed;
}

function requestLevelCal(action) {
  if (state.armed) {
    flashHint("Disarm before setting the level zero-point");
    return;
  }
  if (!state.wsConnected || state.ws.readyState !== WebSocket.OPEN) return;
  state.ws.send(JSON.stringify({ type: "level_cal", action }));
  state.levelMsg = action === "reset" ? "Erasing…" : "Capture requested…";
  state.levelMsgAt = performance.now();
}

document.getElementById("level-btn").addEventListener("click", () => requestLevelCal("start"));
document.getElementById("level-erase-btn").addEventListener("click", () => {
  if (confirm("Erase the stored level zero-point?\n\nThe drone goes back to trusting the sensor board's own idea of level.")) {
    requestLevelCal("reset");
  }
});

function tickPacketAge() {
  const el = document.getElementById("age-val");
  if (!state.lastTelemetryAt) {
    el.textContent = "—";
  } else {
    const ageMs = performance.now() - state.lastTelemetryAt;
    el.textContent = `${(ageMs / 1000).toFixed(1)}s`;
    el.style.color = ageMs > 1000 ? "var(--bad)" : "";
  }
  setTimeout(tickPacketAge, 200);
}

async function pollStatus() {
  try {
    const res = await fetch("/status");
    const s = await res.json();
    document.getElementById("clients-val").textContent = s.clients;
  } catch { /* server not reachable yet - ignore */ }
  setTimeout(pollStatus, 2000);
}

// ---------------------------------------------------------------------
// Shutdown
// ---------------------------------------------------------------------

document.getElementById("shutdown-btn").addEventListener("click", async () => {
  if (state.armed) {
    flashHint("Disarm before shutting down");
    return;
  }
  const ok = confirm(
    "Shut down the Pi 5?\n\nThis cleanly powers off the flight computer. " +
    "Wait for its lights to stop before cutting power to the drone."
  );
  if (!ok) return;

  const btn = document.getElementById("shutdown-btn");
  btn.disabled = true;
  btn.textContent = "SHUTTING DOWN…";
  try {
    await fetch("/shutdown", { method: "POST" });
  } catch { /* connection drops as the Pi goes down - expected */ }
});

// ---------------------------------------------------------------------
// Arm / disarm
// ---------------------------------------------------------------------

function disarm() {
  state.armed = false;
  // Drop the held throttle to zero and centre the sticks, so re-arming
  // always starts from idle rather than resuming a hover setting.
  state.throttle = 0;
  stickResets.forEach((zero) => zero());
  document.getElementById("arm-toggle").classList.remove("armed");
  document.getElementById("arm-toggle").textContent = "ARM";
  document.getElementById("arm-banner").classList.remove("armed");
  document.getElementById("arm-banner").textContent = "DISARMED";
}

function tryArm() {
  if (state.left.y > 0.03) {
    flashHint("Throttle must be at zero to arm");
    return;
  }
  state.armed = true;
  document.getElementById("arm-toggle").classList.add("armed");
  document.getElementById("arm-toggle").textContent = "DISARM";
  document.getElementById("arm-banner").classList.add("armed");
  document.getElementById("arm-banner").textContent = "ARMED";
}

function flashHint(text) {
  const hint = document.querySelector(".hint");
  const original = hint.textContent;
  hint.textContent = text;
  hint.style.color = "var(--bad)";
  setTimeout(() => { hint.textContent = original; hint.style.color = ""; }, 1200);
}

document.getElementById("arm-toggle").addEventListener("click", () => {
  if (state.armed) disarm(); else tryArm();
});
document.getElementById("estop").addEventListener("click", disarm);
window.addEventListener("keydown", (e) => {
  if (e.code === "Space") { e.preventDefault(); disarm(); }
});
window.addEventListener("blur", disarm);
document.addEventListener("visibilitychange", () => {
  if (document.hidden) disarm();
});

// ---------------------------------------------------------------------
// Virtual sticks (pointer events - works for mouse and touch alike)
// ---------------------------------------------------------------------

// Reset hooks for every stick, so disarm() can force them all back to
// idle - important now that the throttle latches.
const stickResets = [];

// A physical transmitter has a gate around the throttle stick; a flat piece
// of glass does not. Dragging "straight up" for throttle always picks up
// some sideways drift, and on the left stick sideways is YAW - which the
// mixer turns into one diagonal motor pair running faster than the other.
//
// So the left stick commits to whichever axis you actually moved along:
// past LOCK_AT deflection it picks the dominant one and zeroes the other
// until you let go. Push up and you get pure throttle; push sideways and
// you get pure yaw. The right stick is NOT locked - roll and pitch
// together is a legitimate, everyday input.
const LOCK_AT = 0.22;

function setupStick(elId, target, opts = {}) {
  const axisLock = !!opts.axisLock;
  const stick = document.getElementById(elId);
  const knob = stick.querySelector(".stick-knob");
  let dragging = false;
  let lockedAxis = null;

  function place(dx, dy) {
    knob.style.transform = `translate(calc(-50% + ${dx}px), calc(-50% + ${dy}px))`;
  }

  // Hard reset to centre.
  function zero() {
    target.x = 0;
    target.y = 0;
    place(0, 0);
  }
  stickResets.push(zero);

  function update(clientX, clientY) {
    const rect = stick.getBoundingClientRect();
    const cx = rect.left + rect.width / 2;
    const cy = rect.top + rect.height / 2;
    let dx = clientX - cx;
    let dy = clientY - cy;
    const mag = Math.hypot(dx, dy);
    // Measured every move rather than cached at setup: the stick and knob
    // both change size when the phone is rotated into landscape, and a
    // stale radius silently rescales the control - full deflection would
    // land at the wrong place, or never be reachable at all.
    const max = Math.max(10, rect.width / 2 - knob.offsetWidth / 2);
    if (mag > max) { dx = (dx / mag) * max; dy = (dy / mag) * max; }

    if (axisLock) {
      // Commit once the pull is clearly in one direction, then stay there
      // for the rest of the drag - so a wobbly thumb can't leak yaw into a
      // throttle push, or throttle into a yaw turn.
      if (lockedAxis === null && mag / max > LOCK_AT) {
        lockedAxis = Math.abs(dx) > Math.abs(dy) ? "x" : "y";
      }
      if (lockedAxis === "x") { dy = 0; }
      else if (lockedAxis === "y") { dx = 0; }
    }

    place(dx, dy);
    target.x = dx / max;
    target.y = -dy / max; // screen Y is inverted vs "up = positive"
  }

  function reset() {
    // Every stick springs back to centre, throttle included. The held
    // throttle lives in state.throttle, not in the knob position.
    zero();
  }

  stick.addEventListener("pointerdown", (e) => {
    dragging = true;
    lockedAxis = null; // each new drag chooses its own axis
    stick.setPointerCapture(e.pointerId);
    update(e.clientX, e.clientY);
  });
  stick.addEventListener("pointermove", (e) => {
    if (dragging) update(e.clientX, e.clientY);
  });
  const end = (e) => {
    dragging = false;
    lockedAxis = null;
    reset();
  };
  stick.addEventListener("pointerup", end);
  stick.addEventListener("pointercancel", end);
}

// ---------------------------------------------------------------------
// Boot
// ---------------------------------------------------------------------

// Left = throttle/yaw, axis-locked so the two can't bleed into each other.
setupStick("stick-left", state.left, { axisLock: true });
setupStick("stick-right", state.right);

// Restore saved trim, wire the trim buttons.
try {
  const r = parseFloat(localStorage.getItem("rollTrim"));
  const p = parseFloat(localStorage.getItem("pitchTrim"));
  if (!Number.isNaN(r)) state.rollTrim = Math.max(-TRIM_LIMIT_DEG, Math.min(TRIM_LIMIT_DEG, r));
  if (!Number.isNaN(p)) state.pitchTrim = Math.max(-TRIM_LIMIT_DEG, Math.min(TRIM_LIMIT_DEG, p));
} catch { /* private mode - start at 0 */ }
[["roll-trim-minus", "roll", -1], ["roll-trim-plus", "roll", 1],
 ["pitch-trim-minus", "pitch", -1], ["pitch-trim-plus", "pitch", 1]].forEach(
  ([id, axis, d]) => document.getElementById(id).addEventListener("click", () => adjustTrim(axis, d)));
document.getElementById("trim-reset").addEventListener("click", () => {
  state.rollTrim = 0; state.pitchTrim = 0;
  try { localStorage.removeItem("rollTrim"); localStorage.removeItem("pitchTrim"); } catch {}
  renderTrim();
});
renderTrim();

disarm();
connectWs();
sendControlLoop();
tickPacketAge();
pollStatus();
