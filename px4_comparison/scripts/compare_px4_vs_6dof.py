#!/usr/bin/env python3
"""Part B: quantitative PX4 SITL vs Quadrotor_6DoF comparison.

IMPORTANT REPORTING RULE: this is a SIMULATION-TO-SIMULATION comparison
(PX4 Gazebo SITL log vs the 6-DoF Simulink plant) under a matched attitude
setpoint sequence. It is NOT physical flight-test validation and must not be
reported as such. Both systems use FRD body / NED earth: NO NED-to-NWU flip.

Prerequisite: models/new_build/run_step7_px4_replay.m must have produced
  px4_comparison/extracted_csv/simulink_6dof_px4_replay.csv
Then run: python3 px4_comparison/scripts/compare_px4_vs_6dof.py
"""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent.parent.parent
PX4 = ROOT / "px4_comparison"
EXTRACTED = PX4 / "extracted_csv"
RESULTS = PX4 / "comparison_results"

ALIGNED_CSV = EXTRACTED / "px4_window_aligned.csv"
SIM_CSV = EXTRACTED / "simulink_6dof_px4_replay.csv"

SIM_NOTE = "SIMULATION-TO-SIMULATION comparison (PX4 SITL vs Simulink) - not flight-test validation"


def step_onset(t: np.ndarray, sp_deg: np.ndarray) -> float:
    """t where the commanded step first exceeds half its total excursion."""
    span = float(np.nanmax(sp_deg) - np.nanmin(sp_deg))
    if span <= 0.5:
        return float(t[0])
    half = np.nanmin(sp_deg) + 0.5 * span
    idx = np.argmax(sp_deg >= half)
    return float(t[idx])


def step_metrics(t: np.ndarray, sp: np.ndarray, actual: np.ndarray,
                 t_step: float, t_end: float):
    """Metrics for one axis over the constant-step hold [t_step, t_end]."""
    post = (t >= t_step) & (t <= t_end)
    tp, sp_post, a_post = t[post], sp[post], actual[post]
    if len(tp) < 10:
        return {k: np.nan for k in ("rmse_vs_sp_deg", "peak_overshoot_pct",
                                    "rise_time_10_90_s", "settling_2pct_s",
                                    "settling_5pct_s", "steady_state_error_deg")}

    rmse = float(np.sqrt(np.mean((a_post - sp_post) ** 2)))
    y0 = float(np.median(sp[~post])) if np.any(~post) else float(sp_post[0])
    yf = float(np.median(sp_post[-max(1, len(sp_post) // 10):]))
    amplitude = yf - y0
    peak = float(np.max(a_post)) if amplitude >= 0 else float(np.min(a_post))

    if abs(amplitude) > 0.5:  # ignore axes with no commanded step
        overshoot = float(max(0.0, (peak - yf) / amplitude * 100))
        lo, hi = y0 + 0.1 * amplitude, y0 + 0.9 * amplitude
        above_lo = np.where((a_post >= min(lo, hi)) & (a_post <= max(lo, hi)))[0]
        t_rise = float(tp[above_lo[-1]] - tp[above_lo[0]]) if len(above_lo) > 1 else np.nan
    else:
        overshoot, t_rise = np.nan, np.nan

    def settling(band_pct: float) -> float:
        if abs(amplitude) <= 0.5:
            return np.nan
        band = abs(amplitude) * band_pct / 100.0
        outside = np.where(np.abs(a_post - yf) > band)[0]
        if len(outside) == 0:
            return 0.0
        return float(tp[outside[-1]] - tp[0])

    steady = float(np.mean(a_post[-max(1, len(a_post) // 10):] - sp_post[-max(1, len(sp_post) // 10):]))
    return {
        "rmse_vs_sp_deg": rmse,
        "peak_overshoot_pct": overshoot,
        "rise_time_10_90_s": t_rise,
        "settling_2pct_s": settling(2),
        "settling_5pct_s": settling(5),
        "steady_state_error_deg": steady,
    }


def main() -> None:
    RESULTS.mkdir(parents=True, exist_ok=True)
    if not SIM_CSV.exists():
        print("=" * 78)
        print("WAITING FOR SIMULINK REPLAY OUTPUT")
        print("=" * 78)
        print(f"Not found: {SIM_CSV}")
        print("On the MATLAB laptop: edit Quadrotor_6DoF.slx (case 7 + From Workspace")
        print("px4_sp_ts), then run models/new_build/run_step7_px4_replay.m.")
        print("Re-run this script after the CSV exists.")
        return

    px4 = pd.read_csv(ALIGNED_CSV)
    sim = pd.read_csv(SIM_CSV)

    # Common grid: intersection of both time spans, referenced to step onset.
    t0 = max(float(px4["t_s"].min()), float(sim["time_s"].min()))
    t1 = min(float(px4["t_s"].max()), float(sim["time_s"].max()))
    t_step = step_onset(px4["t_s"].to_numpy(), px4["phi_sp_deg"].to_numpy())
    # End of the constant-amplitude hold: first return below half amplitude
    sp_roll_all = px4["phi_sp_deg"].to_numpy()
    half = 0.5 * (np.nanmax(sp_roll_all) + np.nanmin(sp_roll_all))
    after = np.where((px4["t_s"].to_numpy() > t_step + 0.5) &
                     (sp_roll_all <= half))[0]
    t_end = float(px4["t_s"].to_numpy()[after[0]]) if len(after) else t_step + 8.0
    t = np.arange(t0, t1, 0.01)
    t_rel = t - t_step
    print(f"Step window: onset t_step = {t_step:.3f} s, hold end = {t_end:.3f} s "
          f"({t_end - t_step:.3f} s hold)")

    def interp(df, time_col, value_col):
        return np.interp(t, df[time_col].to_numpy(), df[value_col].to_numpy())

    sp_roll = interp(px4, "t_s", "phi_sp_deg")
    sp_pitch = interp(px4, "t_s", "theta_sp_deg")
    # Yaw is reported RELATIVE to the window-start heading: the Simulink replay
    # normalises the absolute PX4 NED heading (~96 deg) so the plant can start at psi=0.
    sp_yaw = interp(px4, "t_s", "psi_sp_deg") - float(px4["psi_sp_deg"].iloc[0])
    px4_roll = interp(px4, "t_s", "phi_px4_deg")
    px4_pitch = interp(px4, "t_s", "theta_px4_deg")
    px4_yaw = interp(px4, "t_s", "psi_px4_deg") - float(px4["psi_px4_deg"].iloc[0])
    sim_roll = interp(sim, "time_s", "phi_sim_deg")
    sim_pitch = interp(sim, "time_s", "theta_sim_deg")
    sim_yaw = interp(sim, "time_s", "psi_sim_deg")

    axes = {
        "Roll (excited)": (sp_roll, px4_roll, sim_roll,
                            px4["p_px4_rad_s"], sim["p_sim_rad_s"], "p_px4_rad_s", "p_sim_rad_s"),
        "Pitch (cross)": (sp_pitch, px4_pitch, sim_pitch,
                           px4["q_px4_rad_s"], sim["q_sim_rad_s"], "q_px4_rad_s", "q_sim_rad_s"),
        "Yaw (cross)": (sp_yaw, px4_yaw, sim_yaw,
                         px4["r_px4_rad_s"], sim["r_sim_rad_s"], "r_px4_rad_s", "r_sim_rad_s"),
    }

    rows = []
    for name, (sp, p4, sm, rate4_series, rate_s_series, c4, cs) in axes.items():
        m4 = step_metrics(t, sp, p4, t_step, t_end)
        ms = step_metrics(t, sp, sm, t_step, t_end)
        t_post = (t >= t_step) & (t <= t_end)
        cross_rmse = float(np.sqrt(np.mean((sm[t_post] - p4[t_post]) ** 2)))
        rate4 = float(np.interp(t[t_post], px4["t_s"], rate4_series).max())
        rates = float(np.interp(t[t_post], sim["time_s"], rate_s_series).max())
        rows.append({
            "Axis": name,
            "PX4_RMSE_vs_SP_deg": m4["rmse_vs_sp_deg"],
            "Sim_RMSE_vs_SP_deg": ms["rmse_vs_sp_deg"],
            "CrossSim_RMSE_deg": cross_rmse,
            "PX4_Peak_Overshoot_pct": m4["peak_overshoot_pct"],
            "Sim_Peak_Overshoot_pct": ms["peak_overshoot_pct"],
            "PX4_RiseTime_10_90_s": m4["rise_time_10_90_s"],
            "Sim_RiseTime_10_90_s": ms["rise_time_10_90_s"],
            "PX4_SettlingTime_2pct_s": m4["settling_2pct_s"],
            "Sim_SettlingTime_2pct_s": ms["settling_2pct_s"],
            "PX4_SettlingTime_5pct_s": m4["settling_5pct_s"],
            "Sim_SettlingTime_5pct_s": ms["settling_5pct_s"],
            "PX4_SteadyStateError_deg": m4["steady_state_error_deg"],
            "Sim_SteadyStateError_deg": ms["steady_state_error_deg"],
            "PX4_PeakRate_rad_s": rate4,
            "Sim_PeakRate_rad_s": rates,
            "Notes": SIM_NOTE,
        })
    metrics = pd.DataFrame(rows)
    metrics.to_csv(RESULTS / "px4_vs_6dof_metrics.csv", index=False)
    print(metrics.to_string(index=False))

    # ---- attitude overlay -------------------------------------------------
    fig, axes_plt = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
    for ax, (name, (sp, p4, sm, *_)) in zip(axes_plt, axes.items()):
        ax.plot(t_rel, sp, "k--", lw=1.5, label="Setpoint")
        ax.plot(t_rel, p4, "b", lw=1.5, label="PX4 SITL")
        ax.plot(t_rel, sm, "r", lw=1.5, label="Simulink 6-DoF")
        ax.axvline(0, color="grey", ls=":", lw=1)
        ax.set_ylabel(f"{name.split()[0]} [deg]")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8, loc="best")
    axes_plt[0].set_title(f"Attitude overlay — {SIM_NOTE}", fontsize=10)
    axes_plt[-1].set_xlabel("Time relative to step onset [s]")
    fig.tight_layout()
    fig.savefig(RESULTS / "attitude_overlay_comparison.png", dpi=300)
    plt.close(fig)

    # ---- angular rates ----------------------------------------------------
    fig, axes_plt = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
    for ax, (name, (_, _, _, r4s, rss, c4, cs)) in zip(axes_plt, axes.items()):
        r4 = np.interp(t, px4["t_s"], r4s)
        rs = np.interp(t, sim["time_s"], rss)
        ax.plot(t_rel, r4, "b", lw=1.4, label="PX4 SITL")
        ax.plot(t_rel, rs, "r", lw=1.4, label="Simulink 6-DoF")
        ax.axhline(0, color="k", lw=0.7)
        ax.axvline(0, color="grey", ls=":", lw=1)
        ax.set_ylabel(f"{c4.split('_')[0]} [rad/s]")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
    axes_plt[0].set_title(f"Body rates — {SIM_NOTE}", fontsize=10)
    axes_plt[-1].set_xlabel("Time relative to step onset [s]")
    fig.tight_layout()
    fig.savefig(RESULTS / "angular_rate_comparison.png", dpi=300)
    plt.close(fig)

    # ---- tracking error + altitude ----------------------------------------
    fig, axes_plt = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    axes_plt[0].plot(t_rel, px4_roll - sp_roll, "b", lw=1.4, label="PX4 roll error")
    axes_plt[0].plot(t_rel, sim_roll - sp_roll, "r", lw=1.4, label="Simulink roll error")
    axes_plt[0].axhline(0, color="k", lw=0.7)
    axes_plt[0].set_ylabel("Roll tracking error [deg]")
    axes_plt[0].legend(fontsize=8)
    alt4 = np.interp(t, px4["t_s"], px4["alt_px4_m"])
    alts = np.interp(t, sim["time_s"], sim["alt_sim_m"])
    axes_plt[1].plot(t_rel, alt4, "b", lw=1.4, label="PX4 altitude (-z_NED)")
    axes_plt[1].plot(t_rel, alts, "r", lw=1.4, label="Simulink altitude (-z_e)")
    axes_plt[1].set(xlabel="Time relative to step onset [s]", ylabel="Altitude [m]")
    axes_plt[1].legend(fontsize=8)
    for ax in axes_plt:
        ax.axvline(0, color="grey", ls=":", lw=1)
        ax.grid(True, alpha=0.3)
    axes_plt[0].set_title(f"Tracking error / altitude hold — {SIM_NOTE}", fontsize=10)
    fig.tight_layout()
    fig.savefig(RESULTS / "tracking_error_comparison.png", dpi=300)
    plt.close(fig)

    print(f"Saved metrics + 3 plots to {RESULTS}/")


if __name__ == "__main__":
    main()
