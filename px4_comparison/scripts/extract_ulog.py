#!/usr/bin/env python3
"""Extract the PX4 SITL roll-step ULog into replay-ready CSV files.

Frame convention (per the project's FRD body / NED earth plant):
  * PX4 logs vehicle_attitude in FRD body and vehicle_local_position in NED.
  * Quadrotor_6DoF.slx uses the SAME FRD body / NED earth convention.
  * Therefore NO legacy NED-to-NWU sign flip is applied anywhere in this script.
  * Quaternions are [w, x, y, z] and are converted with the standard ZYX
    (roll-pitch-yaw) formulas used by PX4's own verification utilities.

Outputs (all under px4_comparison/extracted_csv/):
  px4_vehicle_attitude.csv, px4_vehicle_attitude_setpoint.csv,
  px4_vehicle_angular_velocity.csv, px4_vehicle_local_position.csv,
  px4_window_aligned.csv, px4_setpoint_for_simulink.csv,
  px4_extracted_sanity_check.png

Run: python3 px4_comparison/scripts/extract_ulog.py
"""

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from pyulog import ULog

ROOT = Path(__file__).resolve().parent.parent.parent
PX4 = ROOT / "px4_comparison"
ULOG_DIR = PX4 / "ulog"
OUT_DIR = PX4 / "extracted_csv"

PRE_STEP_S = 1.0        # hover trim before the step
POST_STEP_S = 5.0       # settling after the step
RESAMPLE_HZ = 100       # uniform grid for Simulink replay
DT = 1.0 / RESAMPLE_HZ


def find_manifest() -> Path | None:
    for candidate in (PX4 / "attitude_step_run.json",
                      PX4 / "comparison_results" / "attitude_step_run.json"):
        if candidate.exists():
            return candidate
    return None


def resolve_ulg(manifest: dict | None) -> Path:
    name = Path(manifest.get("ulog", "")).name if manifest else ""
    candidates = [ULOG_DIR / name] if name else []
    candidates += sorted(ULOG_DIR.glob("*.ulg"))
    candidates = [p for p in candidates if p.exists()]
    if not candidates:
        raise FileNotFoundError(f"No .ulg file found in {ULOG_DIR}")
    # Prefer the manifest log; otherwise the newest by mtime
    return candidates[0] if name and candidates[0].exists() else max(candidates, key=lambda p: p.stat().st_mtime)


def quat_to_zyx_deg(q: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """[w,x,y,z] quaternion array (N,4) -> roll, pitch, yaw in degrees (ZYX)."""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    phi = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    theta = np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0)))
    psi = np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    return phi, theta, psi


def topic_frame(ulog: ULog, name: str) -> tuple[np.ndarray, dict]:
    ds = ulog.get_dataset(name)
    raw = ds.data
    t_us = np.asarray(raw["timestamp"], dtype=float)
    return t_us / 1e6, raw


def raw_topic_to_csv(t_s: np.ndarray, raw: dict, filename: str) -> pd.DataFrame:
    df = pd.DataFrame({k: np.asarray(v) for k, v in raw.items()})
    df.insert(0, "timestamp_s", t_s)
    df = df.rename(columns={"timestamp": "timestamp_us"})
    df.to_csv(OUT_DIR / filename, index=False)
    return df


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    manifest_path = find_manifest()
    manifest = json.loads(manifest_path.read_text()) if manifest_path else None
    ulg_path = resolve_ulg(manifest)
    ulog = ULog(str(ulg_path))

    boot_us = float(ulog.msg_info_dict.get("boot_time_utc_us", 0))
    print("=" * 78)
    print("PX4 SITL ULOG EXTRACTION — FRD body / NED earth (NO sign flip)")
    print("=" * 78)
    print(f"ULog            : {ulg_path}")
    print(f"Manifest        : {manifest_path}")
    print(f"boot_time_utc_us: {boot_us:.0f}")
    if manifest:
        print(f"Experiment      : {manifest.get('experiment')} | vehicle "
              f"{manifest.get('vehicle')} | roll {manifest['params']['roll_deg']:+.1f} deg "
              f"| hold {manifest['params']['hold_s']:.1f} s")

    # ---- raw topics -------------------------------------------------------
    t_att, att = topic_frame(ulog, "vehicle_attitude")
    t_sp, sp = topic_frame(ulog, "vehicle_attitude_setpoint")
    t_rate, rate = topic_frame(ulog, "vehicle_angular_velocity")
    t_lpos, lpos = topic_frame(ulog, "vehicle_local_position")
    raw_topic_to_csv(t_att, att, "px4_vehicle_attitude.csv")
    raw_topic_to_csv(t_sp, sp, "px4_vehicle_attitude_setpoint.csv")
    raw_topic_to_csv(t_rate, rate, "px4_vehicle_angular_velocity.csv")
    raw_topic_to_csv(t_lpos, lpos, "px4_vehicle_local_position.csv")
    print(f"Raw topic CSVs written: {len(t_att)} att / {len(t_sp)} sp / "
          f"{len(t_rate)} rate / {len(t_lpos)} local-position samples")

    q_att = np.column_stack([att[f"q[{i}]"] for i in range(4)])
    q_sp = np.column_stack([sp[f"q_d[{i}]"] for i in range(4)])
    phi_a, theta_a, psi_a = quat_to_zyx_deg(q_att)
    phi_s, theta_s, psi_s = quat_to_zyx_deg(q_sp)
    p = np.asarray(rate["xyz[0]"], dtype=float)
    q_ = np.asarray(rate["xyz[1]"], dtype=float)
    r = np.asarray(rate["xyz[2]"], dtype=float)
    altitude = -np.asarray(lpos["z"], dtype=float)  # NED z is down -> altitude = -z

    # ---- maneuver window --------------------------------------------------
    cmd_roll = float(manifest["params"]["roll_deg"]) if manifest else 10.0
    step_epoch = float(manifest["phases"]["step"]["start_epoch"]) if manifest else None
    end_epoch = float(manifest["phases"]["step"]["end_epoch"]) if manifest else None
    if boot_us and step_epoch and end_epoch:
        step_start = (step_epoch * 1e6 - boot_us) / 1e6
        step_end = (end_epoch * 1e6 - boot_us) / 1e6
        source = "manifest epoch timestamps + boot_time_utc_us"
    else:
        # auto-detect the longest contiguous commanded-roll hold
        mask = np.abs(phi_s - cmd_roll) < 1.5
        edges = np.diff(np.r_[False, mask, False].astype(int))
        starts, ends = np.where(edges == 1)[0], np.where(edges == -1)[0]
        i = int(np.argmax(t_sp[ends - 1] - t_sp[starts]))
        step_start, step_end = float(t_sp[starts[i]]), float(t_sp[ends[i] - 1])
        source = "auto-detected from vehicle_attitude_setpoint roll"

    w0 = max(float(t_sp[0]), step_start - PRE_STEP_S)
    w1 = min(float(t_sp[-1]), step_end + POST_STEP_S)
    t_grid = np.arange(0.0, w1 - w0, DT)
    t_abs = w0 + t_grid

    print(f"Window source     : {source}")
    print(f"Step window (log) : {step_start:.3f} .. {step_end:.3f} s "
          f"(duration {step_end - step_start:.3f} s)")
    print(f"Aligned window    : {w0:.3f} .. {w1:.3f} s -> t_s = 0 .. {t_grid[-1]:.2f} s "
          f"({len(t_grid)} samples @ {RESAMPLE_HZ} Hz)")
    print(f"Step onset in aligned time: t_step = {step_start - w0:.3f} s")

    def interp(t_src, values, t_query=t_abs):
        return np.interp(t_query, t_src, values)

    aligned = pd.DataFrame({
        "t_s": t_grid,
        "phi_sp_rad": np.deg2rad(interp(t_sp, phi_s)),
        "theta_sp_rad": np.deg2rad(interp(t_sp, theta_s)),
        "psi_sp_rad": np.deg2rad(interp(t_sp, psi_s)),
        "phi_px4_rad": np.deg2rad(interp(t_att, phi_a)),
        "theta_px4_rad": np.deg2rad(interp(t_att, theta_a)),
        "psi_px4_rad": np.deg2rad(interp(t_att, psi_a)),
        "p_px4_rad_s": interp(t_rate, p),
        "q_px4_rad_s": interp(t_rate, q_),
        "r_px4_rad_s": interp(t_rate, r),
        "alt_px4_m": interp(t_lpos, altitude),
    })
    deg_cols = {
        "phi_sp_deg": np.degrees(aligned["phi_sp_rad"]),
        "theta_sp_deg": np.degrees(aligned["theta_sp_rad"]),
        "psi_sp_deg": np.degrees(aligned["psi_sp_rad"]),
        "phi_px4_deg": np.degrees(aligned["phi_px4_rad"]),
        "theta_px4_deg": np.degrees(aligned["theta_px4_rad"]),
        "psi_px4_deg": np.degrees(aligned["psi_px4_rad"]),
    }
    for name, values in deg_cols.items():
        aligned[name] = values
    aligned.to_csv(OUT_DIR / "px4_window_aligned.csv", index=False)

    simulink_input = aligned[["t_s", "phi_sp_rad", "theta_sp_rad", "psi_sp_rad"]].copy()
    simulink_input = simulink_input.rename(columns={"t_s": "time_s"})
    simulink_input.to_csv(OUT_DIR / "px4_setpoint_for_simulink.csv", index=False)

    # ---- MANDATORY frame / sign verification ------------------------------
    print("-" * 78)
    print("FRAME / SIGN / UNIT VERIFICATION (no NED-to-NWU flip)")
    print("-" * 78)
    step_mask = (t_grid >= step_start - w0) & (t_grid <= step_end - w0)
    mid_mask = (t_grid >= step_start - w0 + 0.15 * (step_end - step_start)) & \
               (t_grid <= step_end - w0 - 0.15 * (step_end - step_start))
    print(f"Timestamp units      : ULog us -> s (divide by 1e6); "
          f"epoch -> log via boot_time_utc_us")
    print(f"Quaternion order     : [w,x,y,z] -> ZYX roll/pitch/yaw (deg)")
    print(f"Attitude units       : rad in aligned CSV; deg columns provided")
    print(f"Body-rate units      : rad/s; local position m; altitude = -z_NED")
    print(f"Commanded roll step  : {cmd_roll:+.1f} deg | measured window median "
          f"{np.median(aligned.loc[step_mask, 'phi_sp_deg']):+.2f} deg")
    print(f"Roll sign check      : actual phi median "
          f"{np.median(aligned.loc[mid_mask, 'phi_px4_deg']):+.2f} deg "
          f"-> {'PASS' if np.sign(cmd_roll) == np.sign(np.median(aligned.loc[mid_mask, 'phi_px4_deg'])) else 'FAIL'}")
    edge = aligned[(t_grid >= step_start - w0 - 0.1) & (t_grid <= step_start - w0 + 0.5)]
    print(f"Roll-rate edge check : peak p = {edge['p_px4_rad_s'].max():+.3f} rad/s "
          f"-> {'PASS' if edge['p_px4_rad_s'].max() > 0 else 'FAIL'}")
    alt_med = float(aligned["alt_px4_m"].median())
    target_alt = float(manifest["params"].get("altitude_m", 2.5)) if manifest else 2.5
    print(f"Altitude check       : median alt = {alt_med:+.3f} m "
          f"-> {'PASS' if alt_med > 0 else 'FAIL'}")
    if alt_med < 0.5 * target_alt:
        print(f"  WARNING: step-window altitude is far below the {target_alt:.1f} m target "
              f"(dist_bottom ~0). The roll step was logged while the vehicle was at/gnear "
              f"ground level; treat altitude-hold comparisons with care or re-fly the run.")
    print("Sign-flip policy     : PX4 FRD/NED == plant FRD/NED -> NO sign inversion applied")
    print("-" * 78)

    # ---- sanity plot ------------------------------------------------------
    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    axes[0].plot(t_grid, aligned["phi_sp_deg"], "k--", lw=1.6, label="Roll setpoint")
    axes[0].plot(t_grid, aligned["phi_px4_deg"], "b", lw=1.6, label="Roll actual (PX4)")
    axes[0].axvline(step_start - w0, color="r", ls=":", lw=1.2, label="step onset")
    axes[0].set_ylabel("Roll [deg]")
    axes[0].legend(fontsize=8)
    axes[1].plot(t_grid, aligned["p_px4_rad_s"], "g", lw=1.4)
    axes[1].set_ylabel("Body rate p [rad/s]")
    axes[2].plot(t_grid, aligned["alt_px4_m"], "tab:orange", lw=1.5)
    axes[2].set(xlabel="Aligned time [s]", ylabel="Altitude [m]")
    for ax in axes:
        ax.grid(True, alpha=0.3)
    fig.suptitle("PX4 extracted roll-step sanity check (simulation log)", fontsize=12)
    fig.tight_layout()
    fig.savefig(OUT_DIR / "px4_extracted_sanity_check.png", dpi=300)
    plt.close(fig)

    print("Files written:")
    for f in sorted(OUT_DIR.iterdir()):
        print(f"  {f.relative_to(ROOT)}  ({f.stat().st_size} bytes)")
    print("=" * 78)


if __name__ == "__main__":
    main()
