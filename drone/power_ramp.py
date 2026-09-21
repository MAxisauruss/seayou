"""
Find the highest CPU load this drone's power supply can actually sustain,
and bring load up in stages so it never gets a sudden step.

Why this exists
---------------
Running YOLO at full tilt rebooted the Pi on 2026-09-12, yet the rail reads
a healthy 5.18-5.23 V at idle. That pattern - fine at rest, dies under load -
points at the supply's transient response rather than its wattage.

A buck converter regulates through a feedback loop with finite bandwidth.
When load jumps from near-idle to four cores at full clock in a few
microseconds, the loop cannot react that fast, so the output sags until it
catches up. If the sag crosses the Pi's brownout threshold it resets. The
size of that sag depends on the SLEW RATE of the load, not just its size -
which is exactly why a supply can be "big enough" on paper and still fail.

So: raise the load in steps, watch for under-voltage after each one, and
stop one step below whatever breaks.

    sudo python3 power_ramp.py --probe        # what can this Pi control?
    sudo python3 power_ramp.py --test         # find the limit (WILL load the CPU)
    sudo python3 power_ramp.py --apply 2      # hold a known-good clock cap

NOTE: this tool caps CPU FREQUENCY and pins load affinity. It deliberately
does NOT take cores offline - on the Pi 5 that is a one-way trip (writing 1
back to /sys/.../online fails with EIO) and only a reboot restores them.

READ THIS BEFORE RUNNING --test
-------------------------------
It deliberately loads the CPU until something gives. On a marginal supply
that means the Pi may reset mid-test. That is the point - but it means:

  * Do not run it while flying. Obviously.
  * Results are written to disk AFTER EVERY STEP, not at the end, so a
    reset does not lose the finding. Re-run it after a reset and read the
    file; the last line recorded is the step that killed it.

What it cannot do
-----------------
This mitigates a transient problem. It does not add current capacity. If
the Pi dies at stage 1, the supply is genuinely too small and no amount of
staging will help - see the notes at the bottom of this file about bulk
capacitance and wiring, which is the real fix.
"""
import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

CPU_ROOT = Path("/sys/devices/system/cpu")
RESULTS = Path("/home/pi/power_ramp_results.jsonl")

def read(path, default=None):
    try:
        return Path(path).read_text().strip()
    except OSError:
        return default


def total_cores() -> int:
    """Physical core count, from sysfs rather than os.cpu_count().

    os.cpu_count() reports cores that are ONLINE. An earlier version of
    this file used it to decide how many cores to restore after a test,
    which is circular: once the test had offlined cpu1-3, the restore read
    "1" and dutifully restored one core. The Pi ran on a single core until
    someone noticed.
    """
    return len([d for d in CPU_ROOT.glob("cpu[0-9]*") if (d / "topology").exists()]) or 4


def load_mask(n: int) -> set:
    """CPU set for a load of `n` cores.

    Cores are NOT taken offline. On the Pi 5 CPU hotplug is effectively
    ONE-WAY: offlining works, but writing 1 back to bring a core online
    fails with EIO and only a reboot restores it. Discovered the hard way.

    Restricting the load's affinity instead achieves the same thing for
    this purpose - the unused cores drop to idle and draw very little -
    while staying completely reversible, and leaving every core available
    to the flight loop.
    """
    return set(range(max(1, min(n, total_cores()))))


def build_stages():
    """Load stages, gentlest first, as (cores, max_freq_khz or None).

    Derived from the hardware rather than hardcoded. The first version of
    this file assumed a 1.0 GHz floor; this Pi's actual minimum is
    1.5 GHz, so those stages were silently clamped and two of them were
    identical - the ramp was coarser than it looked.

    Frequency is stepped as well as core count because the Pi 5 scales
    core VOLTAGE with clock, and power goes roughly with V^2*f. That makes
    the clock a bigger lever than core count, not a smaller one.
    """
    f = CPU_ROOT / "cpu0" / "cpufreq"
    try:
        lo = int(read(f / "cpuinfo_min_freq"))
        hi = int(read(f / "cpuinfo_max_freq"))
    except (TypeError, ValueError):
        lo, hi = 1_500_000, 2_400_000
    mid = lo + (hi - lo) // 2
    total = total_cores()
    stages = []
    for cores in range(1, total + 1):
        for khz in (lo, mid):
            stages.append((cores, khz))
    stages.append((total, None))     # None = unrestricted hardware maximum
    return stages


STAGES = build_stages()

#: Seconds to hold each stage. Long enough for thermals and the supply to
#: settle; a step that only fails after a minute is still a failure.
HOLD_S = 12

#: Rail voltage below which a dip counts as dangerous whatever the throttle
#: bits say. Pi 5 brownout sits around 4.8 V, so this stops just short of it
#: rather than waiting for the reset to prove the point.
RAIL_FLOOR_V = 4.90


def throttled() -> dict:
    """Decode vcgencmd get_throttled.

    The LOW bits are live right now; the HIGH bits are sticky since boot.
    Both matter here: live tells you the current stage is failing, sticky
    tells you an earlier one did.
    """
    out = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True,
                         text=True).stdout.strip()
    try:
        value = int(out.split("=")[1], 16)
    except (IndexError, ValueError):
        return {"raw": out, "error": "could not parse"}
    return {
        "raw": out,
        "under_voltage_now": bool(value & 0x1),
        "freq_capped_now": bool(value & 0x2),
        "throttled_now": bool(value & 0x4),
        "under_voltage_since_boot": bool(value & 0x10000),
        "throttled_since_boot": bool(value & 0x40000),
    }


def rail_voltage():
    """Input rail in volts, or None. This is the number that actually
    matters - core voltage is regulated and will look fine right up until
    the moment the input collapses."""
    out = subprocess.run(["vcgencmd", "pmic_read_adc"], capture_output=True,
                         text=True).stdout
    for line in out.splitlines():
        if "EXT5V_V" in line:
            try:
                return float(line.split("=")[1].rstrip("V"))
            except (IndexError, ValueError):
                return None
    return None


def probe() -> dict:
    """What can actually be controlled here? Nothing is assumed."""
    info = {"nproc": os.cpu_count(), "cores_present": total_cores(),
            "hotplug": {}, "cpufreq": {}}
    for cpu in range(1, info["cores_present"]):
        p = CPU_ROOT / f"cpu{cpu}" / "online"
        info["hotplug"][f"cpu{cpu}"] = "writable" if os.access(p, os.W_OK) else (
            "present, needs root" if p.exists() else "not supported")
    f = CPU_ROOT / "cpu0" / "cpufreq"
    if f.is_dir():
        info["cpufreq"] = {
            "governor": read(f / "scaling_governor"),
            "cur_khz": read(f / "scaling_cur_freq"),
            "min_khz": read(f / "cpuinfo_min_freq"),
            "max_khz": read(f / "cpuinfo_max_freq"),
            "governors": read(f / "scaling_available_governors"),
            "max_writable": os.access(f / "scaling_max_freq", os.W_OK),
        }
    info["throttled"] = throttled()
    info["rail_v"] = rail_voltage()
    return info


def set_max_freq(khz) -> str:
    total = total_cores()
    target = khz or read(CPU_ROOT / "cpu0" / "cpufreq" / "cpuinfo_max_freq")
    if target is None:
        return "cpufreq unavailable"
    ok = 0
    for cpu in range(total):
        p = CPU_ROOT / f"cpu{cpu}" / "cpufreq" / "scaling_max_freq"
        try:
            p.write_text(str(target))
            ok += 1
        except OSError:
            pass
    return f"max_freq={target} kHz on {ok} cpu(s)"


def record(entry: dict):
    """Append immediately and fsync.

    The whole point is to survive the reset this test may cause, so the
    finding must be on the card before the next step runs - buffering it
    would lose exactly the line that matters.
    """
    try:
        with open(RESULTS, "a") as fh:
            fh.write(json.dumps(entry) + "\n")
            fh.flush()
            os.fsync(fh.fileno())
    except OSError as e:
        print(f"  (could not write results: {e})")


def burn(cores: int, seconds: float) -> dict:
    """Load `cores` cores and watch the rail while it runs."""
    procs = []
    code = ("import time\n"
            "t=time.monotonic()\n"
            "x=0.0\n"
            f"while time.monotonic()-t < {seconds + 1}:\n"
            "    x += 1.000001 ** 1.5\n")
    mask = load_mask(cores)
    for _ in range(cores):
        proc = subprocess.Popen([sys.executable, "-c", code],
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL)
        try:
            os.sched_setaffinity(proc.pid, mask)
        except OSError:
            pass          # affinity is an optimisation, not a requirement
        procs.append(proc)
    worst_v, samples = None, []
    end = time.monotonic() + seconds
    try:
        while time.monotonic() < end:
            v = rail_voltage()
            if v is not None:
                samples.append(v)
                worst_v = v if worst_v is None else min(worst_v, v)
            th = throttled()
            if th.get("under_voltage_now"):
                break
            time.sleep(0.5)
    finally:
        for p in procs:
            p.kill()
    return {
        "min_rail_v": round(worst_v, 3) if worst_v is not None else None,
        "mean_rail_v": round(sum(samples) / len(samples), 3) if samples else None,
        "throttled": throttled(),
    }


def run_test(args):
    print("Staged power ramp. The Pi may reset - results are saved after every "
          "step.\n")
    print(f"Results file: {RESULTS}\n")
    start = probe()
    record({"event": "test_start", "t": time.time(), "probe": start})

    # Baseline the sticky throttle bits before any load is applied.
    base = start.get("throttled", {})
    base_uv = bool(base.get("under_voltage_since_boot"))
    base_th = bool(base.get("throttled_since_boot"))
    if base_uv or base_th:
        print("Note: under-voltage/throttling was ALREADY flagged before this")
        print("      test began - it happened during BOOT. Baselining those")
        print("      bits; only new events count as a stage failing.")
        print()

    last_good = None
    for i, (cores, khz) in enumerate(STAGES, start=1):
        label = f"stage {i}/{len(STAGES)}: {cores} core(s) @ {khz or 'max'} kHz"
        print(f"{label} ...", flush=True)
        # Raise the CEILING before adding load, so the step is the load and
        # not the clock jumping at the same moment.
        fr = set_max_freq(khz)
        cr = f"load pinned to cores {sorted(load_mask(cores))}"
        time.sleep(1.0)
        result = burn(cores, HOLD_S)

        entry = {"event": "stage", "t": time.time(), "stage": i, "cores": cores,
                 "max_khz": khz, "applied": {"freq": fr, "cores": cr}, **result}

        th = result["throttled"]
        # Only NEW events count. The sticky since-boot bits were already set
        # on this Pi by a brownout during boot, long before any test load -
        # treating them as failure condemned stage 1 for something unrelated.
        live_uv = bool(th.get("under_voltage_now"))
        new_uv = bool(th.get("under_voltage_since_boot")) and not base_uv
        new_th = bool(th.get("throttled_since_boot")) and not base_th
        low_rail = (result["min_rail_v"] is not None
                    and result["min_rail_v"] < RAIL_FLOOR_V)
        bad = live_uv or new_uv or new_th or low_rail
        why = ", ".join(w for w, c in (
            ("under-voltage now", live_uv),
            ("new under-voltage flag", new_uv),
            ("new throttle flag", new_th),
            ("rail below floor", low_rail)) if c) or "none"
        entry["verdict"] = {"failed": bad, "why": why}
        record(entry)

        print(f"   min rail {result['min_rail_v']} V  mean {result['mean_rail_v']} V"
              f"  -> {'FAILED (' + why + ')' if bad else 'ok'}")
        if bad:
            print(f"\n>>> Stage {i} failed: {why}. Last good stage: {last_good}")
            record({"event": "limit_found", "failed_stage": i,
                    "last_good": last_good, "why": why})
            break
        last_good = i
    else:
        print("\n>>> Completed every stage with no under-voltage.")
        record({"event": "all_stages_passed"})

    # Only the clock needs restoring - no core was ever taken offline, so
    # there is nothing to bring back (and on the Pi 5 it could not be).
    print("\nRestoring full clock.")
    set_max_freq(None)


def available_freqs() -> list:
    """Discrete frequencies the governor will accept, ascending."""
    raw = read(CPU_ROOT / "cpu0" / "cpufreq" / "scaling_available_frequencies")
    if raw:
        try:
            return sorted(int(x) for x in raw.split())
        except ValueError:
            pass
    lo = int(read(CPU_ROOT / "cpu0" / "cpufreq" / "cpuinfo_min_freq") or 1_500_000)
    hi = int(read(CPU_ROOT / "cpu0" / "cpufreq" / "cpuinfo_max_freq") or 2_400_000)
    return [lo + (hi - lo) * i // 9 for i in range(10)]


def ramp_to_max(seconds: float, soak_s: float, cores: int) -> dict:
    """Hold a full load while walking the clock ceiling up to maximum.

    The question this answers: can the supply reach full clock if it is
    never asked to make a sudden jump? A buck converter's feedback loop
    can follow a slow change it cannot follow a fast one, so if the
    failure is transient rather than a capacity ceiling, a long enough
    ramp should get there.

    The soak afterwards is the important half. Arriving at maximum proves
    only that the ramp was gentle enough; STAYING there proves the supply
    can actually deliver the steady-state current. A run that reaches
    2.4 GHz and then sags during the soak has found a capacity limit, and
    no amount of extra ramp time will fix that.
    """
    freqs = available_freqs()
    steps = len(freqs)
    dwell = seconds / max(1, steps - 1)

    set_max_freq(freqs[0])
    time.sleep(1.0)

    # Load first, then ramp. Starting the load at the low clock means the
    # only thing changing during the ramp is frequency.
    code = ("import time\n"
            "t=time.monotonic()\n"
            "x=0.0\n"
            f"while time.monotonic()-t < {seconds + soak_s + 8}:\n"
            "    x += 1.000001 ** 1.5\n")
    mask = load_mask(cores)
    procs = []
    for _ in range(cores):
        p = subprocess.Popen([sys.executable, "-c", code],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            os.sched_setaffinity(p.pid, mask)
        except OSError:
            pass
        procs.append(p)

    base = throttled()
    base_uv = bool(base.get("under_voltage_since_boot"))
    ramp_min, soak_min = None, None
    reached = freqs[0]
    failed_at = None
    trace = []

    def sample(bucket):
        nonlocal ramp_min, soak_min
        v = rail_voltage()
        if v is None:
            return True
        if bucket == "ramp":
            ramp_min = v if ramp_min is None else min(ramp_min, v)
        else:
            soak_min = v if soak_min is None else min(soak_min, v)
        th = throttled()
        new_uv = bool(th.get("under_voltage_since_boot")) and not base_uv
        return not (th.get("under_voltage_now") or new_uv or v < RAIL_FLOOR_V)

    try:
        time.sleep(2.0)                      # settle at the starting clock
        for khz in freqs:
            set_max_freq(khz)
            reached = khz
            end = time.monotonic() + dwell
            worst = None
            while time.monotonic() < end:
                v = rail_voltage()
                if v is not None:
                    worst = v if worst is None else min(worst, v)
                if not sample("ramp"):
                    failed_at = khz
                    break
                time.sleep(0.25)
            trace.append({"khz": khz, "min_v": round(worst, 3) if worst else None})
            if failed_at:
                break

        if not failed_at:
            end = time.monotonic() + soak_s
            while time.monotonic() < end:
                if not sample("soak"):
                    failed_at = reached
                    break
                time.sleep(0.25)
    finally:
        for p in procs:
            p.kill()
        set_max_freq(freqs[0])               # back to the known-safe cap

    return {
        "ramp_seconds": seconds, "soak_seconds": soak_s, "cores": cores,
        "reached_khz": reached, "failed_at_khz": failed_at,
        "ok": failed_at is None and reached == freqs[-1],
        "ramp_min_v": round(ramp_min, 3) if ramp_min else None,
        "soak_min_v": round(soak_min, 3) if soak_min else None,
        "trace": trace,
        "throttled": throttled(),
    }


def run_ramp_sweep(args):
    """Try progressively slower ramps until one reaches and holds maximum."""
    durations = [float(x) for x in args.ramp_sweep.split(",")]
    print(f"Ramping {args.cores} loaded cores from "
          f"{available_freqs()[0]//1000} to {available_freqs()[-1]//1000} MHz.")
    print(f"Durations to try: {', '.join(str(int(d)) + 's' for d in durations)}")
    print(f"Soak at maximum after each: {args.soak:.0f}s\n")

    for seconds in durations:
        print(f"--- ramp over {seconds:.0f}s ---", flush=True)
        result = ramp_to_max(seconds, args.soak, args.cores)
        record({"event": "ramp", "t": time.time(), **result})
        reached_mhz = result["reached_khz"] // 1000
        print(f"    reached {reached_mhz} MHz"
              f"   ramp min {result['ramp_min_v']} V"
              f"   soak min {result['soak_min_v']} V")
        if result["ok"]:
            print(f"\n>>> SUCCESS at {seconds:.0f}s: held maximum clock through "
                  f"a {args.soak:.0f}s soak.")
            record({"event": "ramp_success", "seconds": seconds, **result})
            return
        print(f"    failed at {(result['failed_at_khz'] or 0)//1000} MHz\n")

    print(">>> No ramp duration reached and held maximum. The supply is hitting "
          "a CAPACITY limit,\n    not just a transient one - slowing the ramp "
          "further will not help.")
    record({"event": "ramp_sweep_exhausted"})


def main():
    ap = argparse.ArgumentParser(description="Staged CPU power ramp for a marginal supply")
    ap.add_argument("--probe", action="store_true", help="report what is controllable")
    ap.add_argument("--test", action="store_true", help="find the limit (loads the CPU)")
    ap.add_argument("--apply", type=int, metavar="STAGE",
                    help=f"hold one stage (1-{len(STAGES)}) and exit")
    ap.add_argument("--restore", action="store_true", help="all cores, full clock")
    ap.add_argument("--ramp-sweep", metavar="SECS,SECS,...",
                    help="hold a full load and walk the clock up to maximum over "
                         "each of these durations, stopping at the first that "
                         "reaches AND holds it. e.g. 10,20,40,60,90,120")
    ap.add_argument("--soak", type=float, default=30.0,
                    help="seconds to hold at maximum after a successful ramp. "
                         "This is the half that matters - arriving proves the "
                         "ramp was gentle enough, staying proves the supply can "
                         "actually deliver the steady-state current.")
    ap.add_argument("--cores", type=int, default=0,
                    help="cores to load during a ramp sweep (default: all)")
    args = ap.parse_args()
    if not args.cores:
        args.cores = total_cores()

    if args.probe:
        print(json.dumps(probe(), indent=2))
        return 0
    if os.geteuid() != 0:
        print("Needs root to change cores or clocks:  sudo python3 power_ramp.py ...")
        return 1
    if args.restore:
        print(set_max_freq(None))
        print(f"{total_cores()} cores present; none are offlined by this tool.")
        return 0
    if args.apply:
        if not 1 <= args.apply <= len(STAGES):
            print(f"stage must be 1-{len(STAGES)}")
            return 1
        cores, khz = STAGES[args.apply - 1]
        print(set_max_freq(khz))
        print(f"(cores are not offlined; pin your workload to "
              f"{sorted(load_mask(cores))} with taskset)")
        print(f"Holding stage {args.apply}: {cores} core(s) @ {khz or 'max'} kHz")
        return 0
    if args.ramp_sweep:
        run_ramp_sweep(args)
        return 0
    if args.test:
        run_test(args)
        return 0
    ap.print_help()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

# ---------------------------------------------------------------------------
# If this finds the Pi dies even at stage 1, staging is not your problem and
# no software change will fix it. In rough order of effectiveness:
#
#   1. BULK CAPACITANCE AT THE PI, not at the converter. A transient is
#      supplied by whatever charge is physically closest. Low-ESR is what
#      matters, not headline microfarads - ESR sets how fast the cap can
#      actually deliver. Several smaller low-ESR caps in parallel beat one
#      large high-ESR one. The existing 1000uF helped "marginally", which is
#      consistent with it being high-ESR or too far from the load.
#   2. SHORTER, THICKER WIRING between converter and Pi. Wire inductance
#      directly opposes a fast current change, and a metre of thin wire can
#      undo a good converter.
#   3. A CONVERTER WITH BETTER TRANSIENT RESPONSE, or simply more headroom -
#      running a converter near its limit is where loop response is worst.
#   4. Separate the Pi's supply from the ESCs. Motor current steps are far
#      larger than anything the CPU does, and if they share a rail the CPU
#      brownout may actually be caused by the motors.
# ---------------------------------------------------------------------------
