#!/usr/bin/env python3
"""Independent analytical verification of the Simscape operating-point sweep.

Closes Steps 2 and 3 of the quadrotor investigation:
  * checks T = CT*rho*n^2*D^4, Q = CQ*rho*n^2*D^5, P_shaft = Q*omega
  * checks the bridge constants kf, km against models/new_build/init_6dof_params.m
  * checks the electrical/mechanical power balance (P_elec > P_shaft)
  * writes propulsion_verification_summary.csv and 5 publication-ready figures

Usage:  python3 propulsion_analysis/verification_calculations.py
"""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# ---------------------------------------------------------------------------
# Configuration (frozen vehicle constants -- see configuration/vehicle_configuration.md)
# ---------------------------------------------------------------------------
HERE = Path(__file__).resolve().parent
SWEEP_CSV = HERE / "operating_point_sweep.csv"
SUMMARY_CSV = HERE / "propulsion_verification_summary.csv"
PLOTS_DIR = HERE / "plots"

RHO = 1.225        # kg/m^3, standard sea-level air density
D_PROP = 0.254     # m, 10-inch propeller diameter
CT = 0.110         # thrust coefficient
CQ = 0.0090        # torque coefficient

# Bridge constants recorded in init_6dof_params.m
KF_REF = 1.420705e-5   # N/(rad/s)^2
KM_REF = 2.952483e-7   # N*m/(rad/s)^2
HOVER_THRUST_PER_ROTOR = 4.905  # N = (2.0 kg * 9.81 m/s^2) / 4

REQUIRED_COLS = [
    "RPM", "n_rev_s", "omega_rad_s", "Thrust_N", "Torque_Nm",
    "Shaft_Power_W", "Motor_Elec_Power_W", "Terminal_Voltage_V",
]


def load_sweep() -> pd.DataFrame:
    df = pd.read_csv(SWEEP_CSV)
    missing = [c for c in REQUIRED_COLS if c not in df.columns]
    if missing:
        raise ValueError(f"Missing required columns in sweep CSV: {missing}")
    return df


def compute_verification(df: pd.DataFrame) -> pd.DataFrame:
    out = df.copy()

    n = out["RPM"] / 60.0                       # rev/s
    omega = 2.0 * np.pi * n                     # rad/s (analytical)

    out["n_analytical_rev_s"] = n
    out["omega_analytical_rad_s"] = omega
    out["Thrust_analytical_N"] = CT * RHO * n**2 * D_PROP**4
    out["Torque_analytical_Nm"] = CQ * RHO * n**2 * D_PROP**5
    out["Shaft_Power_analytical_W"] = out["Torque_analytical_Nm"] * omega

    out["Thrust_error_pct"] = (
        (out["Thrust_N"] - out["Thrust_analytical_N"]) / out["Thrust_analytical_N"] * 100
    )
    out["Torque_error_pct"] = (
        (out["Torque_Nm"] - out["Torque_analytical_Nm"]) / out["Torque_analytical_Nm"] * 100
    )
    out["Shaft_Power_error_pct"] = (
        (out["Shaft_Power_W"] - out["Shaft_Power_analytical_W"])
        / out["Shaft_Power_analytical_W"] * 100
    )

    out["Efficiency_total"] = out["Shaft_Power_W"] / out["Motor_Elec_Power_W"]
    out["Power_balance_W"] = out["Motor_Elec_Power_W"] - out["Shaft_Power_W"]
    out["P_elec_gt_P_shaft"] = out["Motor_Elec_Power_W"] > out["Shaft_Power_W"]
    return out


def bridge_constants() -> tuple[float, float]:
    kf = (CT * RHO * D_PROP**4) / (4 * np.pi**2)
    km = (CQ * RHO * D_PROP**5) / (4 * np.pi**2)
    return kf, km


def plot_all(df: pd.DataFrame) -> None:
    PLOTS_DIR.mkdir(exist_ok=True)
    rpm = df["RPM"].to_numpy()

    # 1. Thrust vs RPM (Simulated vs Analytical)
    fig, ax = plt.subplots(figsize=(8, 5.2))
    ax.plot(rpm, df["Thrust_N"], "o-", lw=2, ms=6, label="Simulated (Simscape)")
    ax.plot(rpm, df["Thrust_analytical_N"], "s--", lw=1.8, ms=5,
            label=r"Analytical $C_T\rho n^2 D^4$")
    ax.axhline(HOVER_THRUST_PER_ROTOR, color="grey", ls=":", lw=1.2,
               label="Hover thrust / rotor (4.905 N)")
    ax.set(xlabel="RPM", ylabel="Single-rotor thrust [N]",
           title="Thrust vs RPM — simulated vs analytical")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(PLOTS_DIR / "thrust_vs_rpm.png", dpi=300)
    plt.close(fig)

    # 2. Torque vs RPM (Simulated vs Analytical)
    fig, ax = plt.subplots(figsize=(8, 5.2))
    ax.plot(rpm, df["Torque_Nm"], "o-", lw=2, ms=6, color="tab:orange",
            label="Simulated (Simscape)")
    ax.plot(rpm, df["Torque_analytical_Nm"], "s--", lw=1.8, ms=5,
            color="tab:red", label=r"Analytical $C_Q\rho n^2 D^5$")
    ax.set(xlabel="RPM", ylabel="Single-rotor torque [N·m]",
           title="Torque vs RPM — simulated vs analytical")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(PLOTS_DIR / "torque_vs_rpm.png", dpi=300)
    plt.close(fig)

    # 3. Power vs Thrust (single rotor) + hover marker
    fig, ax = plt.subplots(figsize=(8, 5.2))
    ax.plot(df["Thrust_N"], df["Shaft_Power_W"], "o-", lw=2, ms=6,
            label="Shaft power (mechanical)")
    ax.plot(df["Thrust_N"], df["Motor_Elec_Power_W"], "s-", lw=2, ms=6,
            label="Electrical power (motor)")
    ax.axvline(HOVER_THRUST_PER_ROTOR, color="grey", ls=":", lw=1.4,
               label="Hover thrust / rotor")
    # Fit the electrical-power curve so the marker sits at the true hover
    # power (~64 W/rotor), not at the first (2.76 W) sweep point.
    p_coef = np.polyfit(df["Thrust_N"], df["Motor_Elec_Power_W"], 3)
    p_hover = float(np.polyval(p_coef, HOVER_THRUST_PER_ROTOR))
    ax.scatter([HOVER_THRUST_PER_ROTOR], [p_hover],
               marker="*", s=220, color="gold", edgecolor="black", zorder=5,
               label=f"Hover operating point ({p_hover:.2f} W)")
    ax.set(xlabel="Single-rotor thrust [N]", ylabel="Power [W]",
           title="Power vs thrust — single rotor")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(PLOTS_DIR / "power_vs_thrust.png", dpi=300)
    plt.close(fig)

    # 4. Battery behaviour (dual axis)
    fig, ax1 = plt.subplots(figsize=(8, 5.2))
    c1 = "tab:blue"
    ax1.set_xlabel("RPM")
    ax1.set_ylabel("Terminal voltage [V]", color=c1)
    ax1.plot(rpm, df["Terminal_Voltage_V"], "o-", color=c1, lw=2, ms=6)
    ax1.tick_params(axis="y", labelcolor=c1)
    ax1.grid(True, alpha=0.3)
    ax2 = ax1.twinx()
    c2 = "tab:red"
    ax2.set_ylabel("Pack current [A]", color=c2)
    ax2.plot(rpm, df["Pack_Current_A"], "s--", color=c2, lw=2, ms=5)
    ax2.tick_params(axis="y", labelcolor=c2)
    ax1.set_title("Battery behaviour — voltage sag & pack current")
    fig.tight_layout()
    fig.savefig(PLOTS_DIR / "battery_behaviour.png", dpi=300)
    plt.close(fig)

    # 5. Combined 2x2 summary
    fig, axes = plt.subplots(2, 2, figsize=(12, 8.5))
    axes[0, 0].plot(rpm, df["Thrust_N"], "o-", lw=1.8, label="Simulated")
    axes[0, 0].plot(rpm, df["Thrust_analytical_N"], "s--", lw=1.5, label="Analytical")
    axes[0, 0].set(title="Thrust vs RPM", xlabel="RPM", ylabel="Thrust [N]")

    axes[0, 1].plot(rpm, df["Torque_Nm"], "o-", lw=1.8, color="tab:orange", label="Simulated")
    axes[0, 1].plot(rpm, df["Torque_analytical_Nm"], "s--", lw=1.5, color="tab:red",
                    label="Analytical")
    axes[0, 1].set(title="Torque vs RPM", xlabel="RPM", ylabel="Torque [N·m]")

    axes[1, 0].plot(df["Thrust_N"], df["Shaft_Power_W"], "o-", lw=1.8, label="Shaft")
    axes[1, 0].plot(df["Thrust_N"], df["Motor_Elec_Power_W"], "s-", lw=1.8, label="Electrical")
    axes[1, 0].axvline(HOVER_THRUST_PER_ROTOR, color="grey", ls=":", lw=1.2)
    axes[1, 0].set(title="Power vs thrust", xlabel="Thrust [N]", ylabel="Power [W]")

    ax = axes[1, 1]
    ax.plot(rpm, df["Terminal_Voltage_V"], "o-", lw=1.8, color="tab:blue", label="Voltage [V]")
    ax.set(title="Battery behaviour", xlabel="RPM", ylabel="Terminal voltage [V]")
    ax.grid(True, alpha=0.3)
    ax2 = ax.twinx()
    ax2.plot(rpm, df["Pack_Current_A"], "s--", lw=1.8, color="tab:red", label="Current [A]")
    ax2.set_ylabel("Pack current [A]", color="tab:red")

    for ax_ in axes.flat:
        ax_.grid(True, alpha=0.3)
        ax_.legend(loc="best", fontsize=8)
    fig.suptitle("Propulsion verification summary", fontsize=14)
    fig.tight_layout()
    fig.savefig(PLOTS_DIR / "propulsion_verification_plots.png", dpi=300)
    plt.close(fig)


def main() -> None:
    df = load_sweep()
    verified = compute_verification(df)
    kf, km = bridge_constants()

    verified.to_csv(SUMMARY_CSV, index=False)
    plot_all(verified)

    # ---- Console verification report --------------------------------------
    print("=" * 72)
    print("PROPULSION VERIFICATION — independent analytical checks")
    print("=" * 72)
    print(f"Sweep points: {len(verified)}  ({verified['RPM'].min():.0f}–{verified['RPM'].max():.0f} RPM)")
    print(f"Thrust error:      max |err| = {verified['Thrust_error_pct'].abs().max():.3f} %")
    print(f"Torque error:      max |err| = {verified['Torque_error_pct'].abs().max():.3f} %")
    print(f"Shaft power error: max |err| = {verified['Shaft_Power_error_pct'].abs().max():.3f} %")
    print("-" * 72)
    print(f"kf computed = {kf:.9e} N/(rad/s)^2 | init_6dof_params.m = {KF_REF:.9e} "
          f"| delta = {abs(kf - KF_REF):.3e}")
    print(f"km computed = {km:.9e} N*m/(rad/s)^2 | init_6dof_params.m = {KM_REF:.9e} "
          f"| delta = {abs(km - KM_REF):.3e}")
    kf_ok = np.isclose(kf, KF_REF, rtol=1e-4)
    km_ok = np.isclose(km, KM_REF, rtol=1e-4)
    print(f"Bridge-constant match: kf={'PASS' if kf_ok else 'FAIL'}, km={'PASS' if km_ok else 'FAIL'}")
    print("-" * 72)
    eta = verified["Efficiency_total"]
    print(f"eta_total range = {eta.min():.4f} – {eta.max():.4f}")
    power_ok = bool(verified["P_elec_gt_P_shaft"].all())
    print(f"P_electrical > P_shaft at every operating point: "
          f"{'PASS' if power_ok else 'FAIL'} ({verified['P_elec_gt_P_shaft'].sum()}/{len(verified)})")
    print("-" * 72)
    print(f"Wrote {SUMMARY_CSV.name} and {len(list(PLOTS_DIR.glob('*.png')))} plots to {PLOTS_DIR}/")
    print("=" * 72)

    # Hard assertions — the script fails loudly if physics is violated.
    assert verified["Thrust_error_pct"].abs().max() < 5.0, "Thrust error exceeds 5 %"
    assert verified["Torque_error_pct"].abs().max() < 5.0, "Torque error exceeds 5 %"
    assert verified["Shaft_Power_error_pct"].abs().max() < 5.0, "Shaft power error exceeds 5 %"
    assert kf_ok and km_ok, "Bridge constants do not match init_6dof_params.m"
    assert power_ok, "Electrical power must exceed shaft power at every point"
    print("ALL VERIFICATION ASSERTIONS PASSED")


if __name__ == "__main__":
    main()
