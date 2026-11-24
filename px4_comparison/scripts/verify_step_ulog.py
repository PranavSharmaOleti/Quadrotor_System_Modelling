#!/usr/bin/env python3
"""verify_step_ulog.py - confirm the logged setpoint sequence matches the
commanded roll attitude step from attitude_step.py.

Checks (self-contained in the ULog; PX4 timestamps are seconds since boot):
  A. a step window exists in vehicle_attitude_setpoint (q_d -> roll) at the
     commanded amplitude, held for >= 60% of the commanded duration
  B. baseline before the step is level (~0 deg)
  C. return after the step is level (~0 deg)
  D. actual attitude (vehicle_attitude.q) tracks the command (same sign,
     median within 5 deg)
  E. vehicle_rates_setpoint.roll spikes at the step edge and settles mid-hold
  F. actuator_motors show differential mixing during the step
  G. (if manifest + boot_time_utc_us available) wall-clock of the step window
     matches the recorded phase timestamps

Usage:
  .venv/bin/python verify_step_ulog.py [--ulg PATH] [--manifest PATH]
                                       [--roll-deg X] [--hold-s S]
"""

import argparse
import glob
import json
import math
import os
import sys
from pathlib import Path

import numpy as np
from pyulog import ULog

BASE_DIR = Path(__file__).resolve().parent
LOG_DIR = BASE_DIR / "px4-data" / "px4" / "rootfs" / "fs" / "log"
DEFAULT_MANIFEST = BASE_DIR / "attitude_step_run.json"

TOL_DEG = 1.5          # setpoint window matching tolerance
ACTUAL_TOL_DEG = 5.0   # actual-vs-commanded tolerance
EDGE_RATE_MIN = 0.30   # rad/s spike required at the step edge
MID_RATE_MAX = 0.25    # rad/s allowed mid-hold (settled)
MOTOR_SPREAD_MIN = 0.02


def newest_ulg() -> str | None:
    files = glob.glob(str(LOG_DIR / "**" / "*.ulg"), recursive=True)
    return max(files, key=os.path.getmtime) if files else None


def quat_to_roll_deg(w, x, y, z):
    return np.degrees(np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)))


def dataset(ulog: ULog, name: str):
    try:
        return ulog.get_dataset(name)
    except Exception:
        return None


def contiguous_runs(ts, mask, max_gap_s=0.25):
    """Return [(start_idx, end_idx_exclusive), ...] where mask is True."""
    runs = []
    start = None
    for i, m in enumerate(mask):
        if m and start is None:
            start = i
        elif not m and start is not None:
            runs.append((start, i))
            start = None
        elif m and start is not None and (ts[i] - ts[i - 1]) > max_gap_s:
            runs.append((start, i - 1))  # gap splits the run; current True starts new
            start = i
    if start is not None:
        runs.append((start, len(mask)))
    return runs


def window_stats(ts, values, t0, t1):
    m = (ts >= t0) & (ts <= t1)
    if not np.any(m):
        return None
    return values[m]


def main() -> int:
    p = argparse.ArgumentParser(description="Verify attitude-step ULog")
    p.add_argument("--ulg", default=None, help="ULog path (default: manifest's, else newest)")
    p.add_argument("--manifest", default=str(DEFAULT_MANIFEST))
    p.add_argument("--roll-deg", type=float, default=None)
    p.add_argument("--hold-s", type=float, default=None)
    args = p.parse_args()

    manifest = None
    if os.path.exists(args.manifest):
        with open(args.manifest) as f:
            manifest = json.load(f)

    ulg_path = args.ulg or (manifest or {}).get("ulog") or newest_ulg()
    if not ulg_path or not os.path.exists(ulg_path):
        print(f"FAIL  no ULog found (looked at: {ulg_path})")
        return 1

    cmd_roll = args.roll_deg
    hold_s = args.hold_s
    if manifest:
        cmd_roll = cmd_roll if cmd_roll is not None else manifest.get("params", {}).get("roll_deg")
        hold_s = hold_s if hold_s is not None else manifest.get("params", {}).get("hold_s")
    if cmd_roll is None or hold_s is None:
        print("FAIL  --roll-deg/--hold-s not given and not found in manifest")
        return 1

    print(f"ULog      : {ulg_path}")
    print(f"commanded : roll step {cmd_roll:+.1f} deg held {hold_s:.1f} s")
    print()

    ulog = ULog(ulg_path)
    results = []  # (id, name, passed, detail)

    # --- load setpoint (q_d -> roll) ---
    sp = dataset(ulog, "vehicle_attitude_setpoint")
    if sp is None:
        print("FAIL  vehicle_attitude_setpoint missing from ULog")
        return 1
    ts = sp.data["timestamp"] / 1e6  # us -> s since PX4 boot
    roll_sp = quat_to_roll_deg(sp.data["q_d[0]"], sp.data["q_d[1]"],
                               sp.data["q_d[2]"], sp.data["q_d[3]"])
    thrust_z = sp.data["thrust_body[2]"]

    # --- A: detect the step window ---
    mask = np.abs(roll_sp - cmd_roll) < TOL_DEG
    runs = contiguous_runs(ts, mask)
    best = None
    for a, b in runs:
        dur = ts[b - 1] - ts[a]
        if best is None or dur > (ts[best[1] - 1] - ts[best[0]]):
            best = (a, b)
    if best is None or (ts[best[1] - 1] - ts[best[0]]) < 0.6 * hold_s:
        det = "no contiguous window matching commanded roll"
        if best:
            det = f"longest match only {ts[best[1]-1]-ts[best[0]]:.1f}s (< {0.6*hold_s:.1f}s)"
        results.append(("A", "step window detected", False, det))
        report(results)
        return 1
    t0, t1 = ts[best[0]], ts[best[1] - 1]
    dur = t1 - t0
    results.append(("A", "step window detected", True,
                    f"[{t0:.2f}s .. {t1:.2f}s] = {dur:.2f}s "
                    f"(commanded {hold_s:.1f}s, tol +/-{TOL_DEG} deg)"))

    # --- B: baseline before the step ---
    base = window_stats(ts, roll_sp, t0 - 3.0, t0 - 0.5)
    if base is None or len(base) < 10:
        results.append(("B", "baseline level before step", False, "insufficient pre-step samples"))
    else:
        med = float(np.median(np.abs(base)))
        results.append(("B", "baseline level before step", med <= TOL_DEG,
                        f"median |roll_sp| = {med:.2f} deg (<= {TOL_DEG})"))

    # --- C: return to level after the step ---
    ret = window_stats(ts, roll_sp, t1 + 0.5, t1 + 3.0)
    if ret is None or len(ret) < 10:
        results.append(("C", "return to level after step", False, "insufficient post-step samples"))
    else:
        med = float(np.median(np.abs(ret)))
        results.append(("C", "return to level after step", med <= TOL_DEG,
                        f"median |roll_sp| = {med:.2f} deg (<= {TOL_DEG})"))

    # --- D: actual attitude tracks the command ---
    att = dataset(ulog, "vehicle_attitude")
    if att is None:
        results.append(("D", "actual attitude tracks command", False, "vehicle_attitude missing"))
    else:
        at = att.data["timestamp"] / 1e6
        roll_a = quat_to_roll_deg(att.data["q[0]"], att.data["q[1]"],
                                  att.data["q[2]"], att.data["q[3]"])
        a0, a1 = t0 + max(0.1, 0.15 * dur), t1 - max(0.1, 0.15 * dur)
        mid = window_stats(at, roll_a, a0, a1)
        if mid is None or len(mid) < 10:
            results.append(("D", "actual attitude tracks command", False, "no samples mid-step"))
        else:
            med = float(np.median(mid))
            sign_ok = (med > 0) == (cmd_roll > 0) or abs(cmd_roll) < 0.5
            err = abs(med - cmd_roll)
            results.append(("D", "actual attitude tracks command",
                            sign_ok and err <= ACTUAL_TOL_DEG,
                            f"median roll = {med:+.2f} deg vs commanded {cmd_roll:+.1f} "
                            f"(err {err:.2f} <= {ACTUAL_TOL_DEG})"))

    # --- E: rate setpoint edge spike + settled mid-hold ---
    rates = dataset(ulog, "vehicle_rates_setpoint")
    if rates is None:
        results.append(("E", "rate setpoint edge/settle", False, "vehicle_rates_setpoint missing"))
    else:
        rt = rates.data["timestamp"] / 1e6
        rr = rates.data["roll"]  # rad/s
        edge = window_stats(rt, rr, t0 - 0.5, t0 + 0.5)
        midr = window_stats(rt, rr, t0 + 0.4 * dur, t0 + 0.8 * dur)
        edge_max = float(np.max(np.abs(edge))) if edge is not None and len(edge) else 0.0
        mid_med = float(np.median(np.abs(midr))) if midr is not None and len(midr) else 99.0
        ok = edge_max >= EDGE_RATE_MIN and mid_med <= MID_RATE_MAX
        results.append(("E", "rate setpoint edge/settle", ok,
                        f"edge max |roll_sp_rate| = {edge_max:.2f} rad/s (>= {EDGE_RATE_MIN}), "
                        f"mid-hold median = {mid_med:.2f} rad/s (<= {MID_RATE_MAX})"))

    # --- F: motor differential mixing during the step ---
    motors = dataset(ulog, "actuator_motors")
    if motors is None:
        results.append(("F", "motor differential during step", False, "actuator_motors missing"))
    else:
        mt = motors.data["timestamp"] / 1e6
        m4 = np.vstack([motors.data[f"control[{i}]"] for i in range(4)])
        spread = np.max(m4, axis=0) - np.min(m4, axis=0)
        step_s = window_stats(mt, spread, t0 + 0.1 * dur, t1 - 0.1 * dur)
        base_s = window_stats(mt, spread, t0 - 3.0, t0 - 0.5)
        sp_step = float(np.mean(step_s)) if step_s is not None and len(step_s) else 0.0
        sp_base = float(np.mean(base_s)) if base_s is not None and len(base_s) else 0.0
        ok = sp_step > MOTOR_SPREAD_MIN
        results.append(("F", "motor differential during step", ok,
                        f"spread(step) = {sp_step:.4f} vs spread(baseline) = {sp_base:.4f} "
                        f"(> {MOTOR_SPREAD_MIN})"))

    # --- thrust sanity (informational + basic bound) ---
    th_win = window_stats(ts, thrust_z, t0, t1)
    th = float(np.median(th_win)) if th_win is not None and len(th_win) else 0.0
    results.append(("G", "thrust setpoint sane during step", 0.1 <= abs(th) <= 1.0,
                    f"median thrust_body[2] = {th:+.3f}"))

    # --- H: wall-clock cross-check vs manifest ---
    if manifest and "step" in manifest.get("phases", {}):
        boot = ulog.msg_info_dict.get("boot_time_utc_us")
        if boot:
            epoch0 = (float(boot) + t0 * 1e6) / 1e6
            expect = manifest["phases"]["step"]["start_epoch"]
            diff = abs(epoch0 - expect)
            results.append(("H", "wall-clock matches manifest", diff <= 3.0,
                            f"ULog step start {epoch0:.3f} vs manifest {expect:.3f} "
                            f"(delta {diff:.2f}s <= 3)"))
        else:
            results.append(("H", "wall-clock matches manifest", False, "boot_time_utc_us missing"))
    else:
        results.append(("H", "wall-clock matches manifest", None, "no manifest phase data (skipped)"))

    return report(results)


def report(results) -> int:
    print(f"{'ID':<3} {'CHECK':<34} {'RESULT':<7} DETAIL")
    print("-" * 100)
    failed = 0
    skipped = 0
    for rid, name, ok, detail in results:
        if ok is None:
            mark = "SKIP"
            skipped += 1
        elif ok:
            mark = "PASS"
        else:
            mark = "FAIL"
            failed += 1
        print(f"{rid:<3} {name:<34} {mark:<7} {detail}")
    print("-" * 100)
    total = len(results) - skipped
    if failed == 0:
        print(f"RESULT: PASS ({total - skipped}/{total} checks passed"
              + (f", {skipped} skipped" if skipped else "") + ")")
        return 0
    print(f"RESULT: FAIL ({failed} of {total} checks failed)")
    return 1


if __name__ == "__main__":
    sys.exit(main())
