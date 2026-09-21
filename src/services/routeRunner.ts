// Flies a multi-waypoint plan by driving the single-target mission API.
//
// The aircraft only understands ONE target at a time (groundstation/
// mission.py: start(lat, lon, alt) -> running -> holding). A route is
// therefore a sequence: send a target, wait for it to arrive, send the
// next. This module owns that sequence and nothing else.
//
// --------------------------------------------------------------------
// WHERE THIS RUNS, AND WHY THAT MATTERS
// --------------------------------------------------------------------
// In the browser. Close the tab mid-route and the sequence stops - the
// aircraft finishes the leg it was given and then holds position, which
// is the safe outcome but is NOT the route continuing. The same is true
// if the laptop sleeps or the WiFi drops.
//
// That is a deliberate limit of this first version, not an oversight.
// Putting the sequencer on the Pi would make a route survive the ground
// station going away, and mission.py's header already names that as the
// next step towards real autonomy. Until then: watch the aircraft.
//
// --------------------------------------------------------------------
// RULES THIS FOLLOWS
// --------------------------------------------------------------------
//   1. It never sends the next leg until the aircraft says it arrived.
//   2. Anything it did not expect stops the run - an abort, a refusal, a
//      mission that went idle underneath it, a leg that timed out. It
//      does not retry and it does not skip ahead on its own. A route that
//      quietly carries on after something went wrong is how an aircraft
//      ends up somewhere nobody chose.
//   3. The pilot always wins. Touching the sticks makes the ground
//      station abort the mission, which this sees and stops on.
import type { Waypoint } from "./flightPlan";
import { droneLink } from "./droneLink";

export type RunState =
  | "idle"
  | "flying"
  | "finished"
  | "stopped";

export interface RunSnapshot {
  state: RunState;
  /** Index into the plan of the leg being flown, or -1. */
  index: number;
  total: number;
  /** Plain-language account of what just happened. Always set. */
  message: string;
  /** The waypoint currently commanded, if any. */
  target: Waypoint | null;
  /** Set when the run stopped for a reason worth acting on. */
  problem: boolean;
}

type Listener = () => void;

/** Mission status as the ground station reports it (see mission.py). */
interface MissionStatus {
  state: "idle" | "running" | "holding" | "takeoff" | "landing" | "aborted";
  reason?: string;
  distance_m?: number | null;
}

const POLL_MS = 700;

/**
 * How long a leg may take before the run stops, as a multiple of the
 * time it should take at cruise. Generous, because wind, climb and the
 * guidance's own caution all stretch a leg - but bounded, so a drone
 * fighting a headwind it cannot beat does not sit there until the
 * battery failsafe deals with it.
 */
const LEG_TIMEOUT_FACTOR = 4;
const LEG_TIMEOUT_FLOOR_MS = 45_000;

class RouteRunner {
  private listeners = new Set<Listener>();
  private snapshot: RunSnapshot = {
    state: "idle",
    index: -1,
    total: 0,
    message: "No route running.",
    target: null,
    problem: false,
  };

  private plan: Waypoint[] = [];
  private timer: number | null = null;
  private legStartedAt = 0;
  private legBudgetMs = 0;
  /** True between sending a target and seeing the aircraft act on it. */
  private awaitingStart = false;

  subscribe = (fn: Listener) => {
    this.listeners.add(fn);
    return () => {
      this.listeners.delete(fn);
    };
  };

  getSnapshot = (): RunSnapshot => this.snapshot;

  private publish(patch: Partial<RunSnapshot>) {
    this.snapshot = { ...this.snapshot, ...patch };
    this.listeners.forEach((fn) => fn());
  }

  /**
   * Start flying a plan. The aircraft must already be airborne and
   * holding - this sends positions, never a takeoff.
   */
  async start(plan: Waypoint[], legBudgets: number[]) {
    if (this.snapshot.state === "flying") return;
    if (plan.length === 0) return;
    this.plan = plan;
    this.legBudgets = legBudgets;
    this.publish({
      state: "flying",
      index: -1,
      total: plan.length,
      message: "Starting route.",
      target: null,
      problem: false,
    });
    await this.sendLeg(0);
    this.poll();
  }

  private legBudgets: number[] = [];

  /** Stop the route AND tell the aircraft to stop. */
  async abort(why = "Stopped by the operator.") {
    this.clearTimer();
    if (this.snapshot.state === "flying") {
      try {
        await fetch("/mission", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ action: "abort" }),
        });
      } catch {
        // The route is stopping either way; say so honestly below.
      }
    }
    this.publish({ state: "stopped", message: why, target: null, problem: false });
  }

  /**
   * Give up on the current leg and go to the next one. Only offered
   * after a run has stopped, so it is always a deliberate choice.
   */
  async skip() {
    const next = this.snapshot.index + 1;
    if (next >= this.plan.length) {
      this.publish({ state: "finished", message: "No legs left to skip to.", target: null });
      return;
    }
    this.publish({ state: "flying", problem: false });
    await this.sendLeg(next);
    this.poll();
  }

  private async sendLeg(i: number) {
    const wp = this.plan[i];
    if (!wp) {
      this.clearTimer();
      this.publish({
        state: "finished",
        index: this.plan.length,
        message: `Route complete - ${this.plan.length} waypoints flown. The aircraft is holding at the last one.`,
        target: null,
      });
      return;
    }
    this.publish({
      index: i,
      target: wp,
      message: `Flying to ${wp.label ?? i + 1} of ${this.plan.length}.`,
    });
    this.legStartedAt = Date.now();
    this.legBudgetMs = Math.max(
      LEG_TIMEOUT_FLOOR_MS,
      (this.legBudgets[i] ?? 60) * 1000 * LEG_TIMEOUT_FACTOR,
    );
    this.awaitingStart = true;

    try {
      const r = await fetch("/mission", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ lat: wp.lat, lon: wp.lon, alt: wp.altM }),
      });
      const d = await r.json().catch(() => ({}));
      if (!r.ok || d?.ok === false) {
        // A refusal is the aircraft's safety limits talking. Surface the
        // reason verbatim - it names the actual problem far better than
        // anything this layer could infer.
        this.stopWithProblem(
          `Leg ${i + 1} refused by the aircraft: ${d?.message ?? `HTTP ${r.status}`}`,
        );
      }
    } catch {
      this.stopWithProblem(
        `Could not reach the ground station to send leg ${i + 1}. The aircraft keeps doing whatever it was already doing.`,
      );
    }
  }

  private stopWithProblem(message: string) {
    this.clearTimer();
    this.publish({ state: "stopped", message, problem: true, target: null });
  }

  private clearTimer() {
    if (this.timer !== null) {
      window.clearTimeout(this.timer);
      this.timer = null;
    }
  }

  private poll = () => {
    this.clearTimer();
    if (this.snapshot.state !== "flying") return;
    this.timer = window.setTimeout(async () => {
      let m: MissionStatus | null = null;
      try {
        const r = await fetch("/status");
        if (r.ok) m = (await r.json())?.mission ?? null;
      } catch {
        // One missed poll is not a reason to stop a flight; the leg
        // timeout below is the backstop if the link is really gone.
      }
      if (this.snapshot.state !== "flying") return;
      if (m) this.consider(m);
      this.poll();
    }, POLL_MS);
  };

  private consider(m: MissionStatus) {
    const i = this.snapshot.index;

    // The aircraft's own low-pack failsafe outranks the route. When it
    // decides to come home there is, by definition, only enough pack to
    // do exactly that - so the rest of the search is over.
    //
    // Note what this does NOT do: it does not send an abort. The Pi flies
    // the return itself as a mission, and aborting would cancel the very
    // thing keeping the aircraft in the air. Stop sequencing, say so
    // plainly, and stay out of the way.
    const guard = droneLink.getSnapshot().telemetry?.battery_guard;
    if (guard && (guard.level === "return" || guard.level === "land_now")) {
      this.clearTimer();
      this.publish({
        state: "stopped",
        problem: true,
        target: null,
        message:
          guard.level === "land_now"
            ? `Route cancelled - the aircraft is landing now on low battery. ${guard.reason}`
            : `Route cancelled - the aircraft is flying home on low battery. ${guard.reason}`,
      });
      return;
    }

    if (m.state === "running" || m.state === "takeoff") {
      this.awaitingStart = false;
      return;
    }

    if (m.state === "aborted") {
      this.stopWithProblem(
        `Route stopped: the aircraft aborted the mission${m.reason ? ` - ${m.reason}` : ""}. ` +
          "If that was the sticks moving, this is working as intended: the pilot has it.",
      );
      return;
    }

    if (m.state === "holding") {
      const arrived =
        (m.reason ?? "").toLowerCase().startsWith("arrived") ||
        (typeof m.distance_m === "number" && m.distance_m <= 3);
      if (arrived) {
        this.awaitingStart = false;
        void this.sendLeg(i + 1);
        return;
      }
      // Holding without having arrived is mission.py's own leg timeout.
      // It could not get there, so the rest of the plan is questionable.
      this.stopWithProblem(
        `Route stopped at leg ${i + 1}: the aircraft is holding without arriving` +
          `${m.reason ? ` - ${m.reason}` : ""}. It is hovering where it got to.`,
      );
      return;
    }

    if (m.state === "idle" && !this.awaitingStart) {
      this.stopWithProblem(
        "Route stopped: the mission went idle unexpectedly. Nothing further has been sent.",
      );
      return;
    }

    if (Date.now() - this.legStartedAt > this.legBudgetMs) {
      this.stopWithProblem(
        `Route stopped: leg ${i + 1} took more than ${Math.round(this.legBudgetMs / 1000)} s. ` +
          "Check for wind, or a target the aircraft cannot reach.",
      );
    }
  }
}

export const routeRunner = new RouteRunner();
