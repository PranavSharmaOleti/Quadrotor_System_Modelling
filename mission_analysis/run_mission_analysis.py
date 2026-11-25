#!/usr/bin/env python3
"""Step 6 system-level studies: power-model fit, payload/margin, parameter
sensitivity, 4-phase mission energy, and battery-capacity sensitivity.

All numbers are anchored to the verified artefacts in this repository:
  - propulsion_analysis/operating_point_sweep.csv   (8-point simlog sweep)
  - flight_dynamics_6dof/test4_payload_equilibrium/test4_payload_summary.csv
  - models/new_build/init_6dof_params.m             (frozen bridge constants)
  - configuration/vehicle_configuration.md          (frozen configuration)
  - report/...blueprint.md                          (Step 6 profile & filenames)

Run:  python3 mission_analysis/run_mission_analysis.py
"""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# ---------------------------------------------------------------------------
# Frozen constants -- must match init_6dof_params.m / vehicle_configuration.md
# ---------------------------------------------------------------------------
ROOT = Path(__file__).resolve().parent.parent
HERE = Path(__file__).resolve().parent
PLOTS = HERE / "plots"

SWEEP_CSV = ROOT / "propulsion_analysis" / "operating_point_sweep.csv"
TEST4_CSV = (ROOT / "flight_dynamics_6dof" / "test4_payload_equilibrium"
             / "test4_payload_summary.csv")

RHO = 1.225          # kg/m^3
D_PROP = 0.254       # m
CT = 0.110           # -
CQ = 0.0090          # -
OMEGA_MAX = 942.48   # rad/s
M_BASE = 2.00        # kg
G = 9.81             # m/s^2
V_NOM = 14.8         # V (4S)
C_BAT = 5.0          # Ah
DOD_USABLE = 0.80    # blueprint: E_usable = 0.80 * V_nom * C_bat
SOC_RESERVE = 20.0   # % LiPo reserve threshold

# Blueprint Section 5.2: exact 4-phase mission profile (not the optional fallback)
MISSION_PHASES = [
    ("Take-off",          5.0,  1.15),
    ("Climb",            30.0,  1.08),
    ("Hover",           600.0,  1.00),
    ("Descent/Landing",  35.0,  0.92),
]


# ---------------------------------------------------------------------------
# Helper functions
# ---------------------------------------------------------------------------
def kf_km(d: float = D_PROP, ct: float = CT, cq: float = CQ) -> tuple[float, float]:
    """Exact bridge constants from the Simscape n-form to the 6-DoF omega-form."""
    kf = ct * RHO * d**4 / (4 * np.pi**2)
    km = cq * RHO * d**5 / (4 * np.pi**2)
    return kf, km


def hover_point(mass: float, kf: float) -> tuple[float, float, float]:
    """Return (omega_hover [rad/s], RPM, per-rotor thrust [N])."""
    t_total = mass * G
    omega = np.sqrt(t_total / (4 * kf))
    return omega, omega * 60 / (2 * np.pi), t_total / 4


def r2_rmse(y: np.ndarray, yhat: np.ndarray) -> tuple[float, float]:
    ss_res = float(np.sum((y - yhat) ** 2))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    return 1.0 - ss_res / ss_tot, float(np.sqrt(np.mean((y - yhat) ** 2)))


def strictly_increasing(grid: np.ndarray, predictor) -> bool:
    values = np.asarray(predictor(grid), dtype=float)
    return bool(np.all(np.diff(values) > 0))


def fit_models(t: np.ndarray, y: np.ndarray):
    """Fit candidate models; return all fits and the best strictly-monotonic one."""
    candidates: dict[str, dict] = {}

    # Physical momentum-theory-motivated form: P = a*T^(3/2) + b*T + c
    X = np.column_stack([t**1.5, t, np.ones_like(t)])
    coef, *_ = np.linalg.lstsq(X, y, rcond=None)
    a, b, c = coef
    phys = lambda x: a * np.asarray(x) ** 1.5 + b * np.asarray(x) + c
    candidates["physical a*T^1.5+b*T+c"] = {
        "predict": phys,
        "coef": (a, b, c),
    }

    # Polynomial alternatives (degree 2 and 3)
    for deg in (2, 3):
        p = np.polyfit(t, y, deg)
        pred = lambda x, p=p: np.polyval(p, x)
        candidates[f"poly{deg}"] = {"predict": pred, "coef": tuple(p)}

    grid = np.linspace(t.min() * 0.9, t.max() * 1.1, 400)
    for name, info in candidates.items():
        info["r2"], info["rmse"] = r2_rmse(y, info["predict"](t))
        info["monotonic"] = strictly_increasing(grid, info["predict"])

    valid = [info for info in candidates.values() if info["monotonic"]]
    if not valid:
        raise RuntimeError("No strictly monotonic fit found for the sweep data")
    best = min(valid, key=lambda v: v["rmse"])
    return candidates, best


def phase_breakdown(mass: float, kf: float, power_predictor, current_predictor,
                    phase_list=MISSION_PHASES) -> pd.DataFrame:
    """Compute the 4-phase electrical energy integration for a given mass."""
    rows, cumulative_wh, cumulative_ah = [], 0.0, 0.0
    for name, duration, mult in phase_list:
        total_thrust = mult * mass * G
        per_rotor = total_thrust / 4
        rpm = np.sqrt(per_rotor / kf) * 60 / (2 * np.pi)
        p_elec_total = 4 * float(power_predictor(per_rotor))
        i_pack = float(current_predictor(per_rotor))
        energy_wh = p_elec_total * duration / 3600.0
        energy_kj = p_elec_total * duration / 1000.0
        cumulative_wh += energy_wh
        cumulative_ah += i_pack * duration / 3600.0
        soc = 100.0 - cumulative_ah / C_BAT * 100.0
        rows.append({
            "Phase": name,
            "Duration_s": duration,
            "Thrust_Multiplier": mult,
            "Total_Thrust_N": total_thrust,
            "Per_Rotor_Thrust_N": per_rotor,
            "Rotor_RPM": rpm,
            "Total_Elec_Power_W": p_elec_total,
            "Pack_Current_A": i_pack,
            "Phase_Energy_Wh": energy_wh,
            "Phase_Energy_kJ": energy_kj,
            "Cumulative_Energy_Wh": cumulative_wh,
            "Cumulative_Ah_Used": cumulative_ah,
            "End_SoC_pct": soc,
        })
    return pd.DataFrame(rows)


# ---------------------------------------------------------------------------
# Main studies
# ---------------------------------------------------------------------------
def main() -> None:
    PLOTS.mkdir(exist_ok=True)
    sweep = pd.read_csv(SWEEP_CSV)
    test4 = pd.read_csv(TEST4_CSV)
    kf, km = kf_km()

    t_sweep = sweep["Thrust_N"].to_numpy()
    p_sweep = sweep["Motor_Elec_Power_W"].to_numpy()   # single-rotor electrical power
    i_sweep = sweep["Pack_Current_A"].to_numpy()       # total pack current

    # ---- 1. Power / current model fits -----------------------------------
    p_fits, p_best = fit_models(t_sweep, p_sweep)
    i_fits, i_best = fit_models(t_sweep, i_sweep)
    p_pred, i_pred = p_best["predict"], i_best["predict"]

    print("=" * 78)
    print("1. ELECTRICAL POWER MODEL FIT (single rotor)")
    print("=" * 78)
    for name, f in p_fits.items():
        flag = "monotonic" if f["monotonic"] else "NON-MONOTONIC"
        print(f"  {name:28s} coef={tuple(round(v, 6) for v in f['coef'])} "
              f"R2={f['r2']:.6f} RMSE={f['rmse']:.4f} [{flag}]")
    print(f"  SELECTED: {list(p_fits.keys())[list(p_fits.values()).index(p_best)]}")
    print(f"  Pack-current fit: {[k for k,v in i_fits.items() if v is i_best][0]} "
          f"R2={i_best['r2']:.6f} RMSE={i_best['rmse']:.4f}")

    t_hover_1 = M_BASE * G / 4
    p_hover_1 = float(p_pred(t_hover_1))
    lo_t = float(sweep.loc[sweep["RPM"] < 5500, "Thrust_N"].iloc[-1])
    hi_t = float(sweep.loc[sweep["RPM"] > 5500, "Thrust_N"].iloc[0])
    lo_p = float(sweep.loc[sweep["RPM"] < 5500, "Motor_Elec_Power_W"].iloc[-1])
    hi_p = float(sweep.loc[sweep["RPM"] > 5500, "Motor_Elec_Power_W"].iloc[0])
    print(f"  Hover T/rotor = {t_hover_1:.4f} N -> fitted P_elec/rotor = {p_hover_1:.3f} W "
          f"(total {4*p_hover_1:.2f} W)")
    print(f"  Bracket check (5000 RPM {lo_t:.3f} N / {lo_p:.2f} W ; "
          f"6000 RPM {hi_t:.3f} N / {hi_p:.2f} W): "
          f"{'PASS' if lo_t < t_hover_1 < hi_t and lo_p < p_hover_1 < hi_p else 'FAIL'}")

    fig, ax = plt.subplots(figsize=(8, 5.4))
    ax.scatter(t_sweep, p_sweep, s=55, color="black", zorder=5, label="Simscape sweep")
    grid = np.linspace(t_sweep.min(), t_sweep.max(), 300)
    ax.plot(grid, p_pred(grid), lw=2.2, label="Selected fit")
    for name, f in p_fits.items():
        if f is not p_best:
            ax.plot(grid, f["predict"](grid), lw=1.1, ls="--", alpha=0.75, label=name)
    ax.scatter([t_hover_1], [p_hover_1], marker="*", s=220, color="gold",
               edgecolor="black", zorder=6, label="Hover (4.905 N)")
    ax.set(xlabel="Single-rotor thrust [N]", ylabel="Single-rotor electrical power [W]",
           title="Electrical power vs thrust — model fit validation")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, loc="best")
    fig.tight_layout()
    fig.savefig(PLOTS / "power_thrust_fit.png", dpi=300)
    plt.close(fig)

    # ---- 2. Payload & thrust-margin study --------------------------------
    print("\n" + "=" * 78)
    print("2. PAYLOAD & THRUST-MARGIN STUDY")
    print("=" * 78)
    t_max_total = 4 * kf * OMEGA_MAX**2
    usable_wh = DOD_USABLE * V_NOM * C_BAT
    payload_rows = []
    for mass in (2.0, 2.5, 3.0):
        omega, rpm, t_per = hover_point(mass, kf)
        shaft_total = 4 * km * omega**3
        elec_total = 4 * float(p_pred(t_per))
        endurance = usable_wh / elec_total * 60
        payload_rows.append({
            "Mass_kg": mass,
            "Payload_Added_kg": mass - M_BASE,
            "Hover_Thrust_Total_N": mass * G,
            "Hover_Thrust_Per_Rotor_N": t_per,
            "Hover_RPM": rpm,
            "Max_Thrust_Total_N": t_max_total,
            "TWR": t_max_total / (mass * G),
            "Thrust_Margin_N": t_max_total - mass * G,
            "Hover_Shaft_Power_Total_W": shaft_total,
            "Hover_Elec_Power_Total_W": elec_total,
            "Hover_Pack_Current_A": float(i_pred(t_per)),
            "Usable_Energy_Wh_80pct": usable_wh,
            "Est_Hover_Endurance_min": endurance,
        })
    payload_df = pd.DataFrame(payload_rows)
    payload_df.to_csv(HERE / "payload_thrust_margin.csv", index=False)

    # Cross-check against Step 5 Test 4 outputs (RPM and TWR must agree < 0.5 %)
    deviations = []
    for _, row in payload_df.iterrows():
        ref = test4[np.isclose(test4["Mass_kg"], row["Mass_kg"])].iloc[0]
        for col in ("Hover_RPM", "TWR"):
            deviations.append(abs(row[col] - ref[col]) / ref[col] * 100)
    max_dev = max(deviations)
    print(f"  Max deviation vs test4_payload_summary.csv (RPM/TWR): {max_dev:.4f} % "
          f"-> {'PASS' if max_dev < 0.5 else 'FAIL'}")
    assert max_dev < 0.5, "Payload study disagrees with verified Step 5 Test 4 outputs"
    print(payload_df[["Mass_kg", "Hover_RPM", "TWR", "Hover_Elec_Power_Total_W",
                      "Est_Hover_Endurance_min"]].to_string(index=False))

    fine = np.linspace(1.5, 3.5, 21)
    fine_rows = []
    for mass in fine:
        omega, rpm, t_per = hover_point(mass, kf)
        elec = 4 * float(p_pred(t_per))
        fine_rows.append({"Mass_kg": mass, "TWR": t_max_total / (mass * G),
                          "Thrust_Margin_N": t_max_total - mass * G,
                          "Hover_Elec_Power_W": elec,
                          "Endurance_min": usable_wh / elec * 60})
    fine_df = pd.DataFrame(fine_rows)

    fig, axes = plt.subplots(2, 2, figsize=(11, 7.8))
    axes[0, 0].plot(fine_df["Mass_kg"], fine_df["TWR"], lw=2)
    axes[0, 0].axhline(2.0, color="r", ls="--", lw=1.2, label="TWR = 2.0 reference")
    axes[0, 0].scatter(payload_df["Mass_kg"], payload_df["TWR"], color="k", zorder=5)
    axes[0, 0].set(title="Thrust-to-weight ratio", ylabel="TWR")
    axes[0, 0].legend(fontsize=8)
    axes[0, 1].plot(fine_df["Mass_kg"], fine_df["Thrust_Margin_N"], lw=2, color="tab:orange")
    axes[0, 1].set(title="Thrust margin", ylabel="Margin [N]")
    axes[1, 0].plot(fine_df["Mass_kg"], fine_df["Hover_Elec_Power_W"], lw=2, color="tab:green")
    axes[1, 0].set(title="Hover electrical power", xlabel="Mass [kg]", ylabel="Power [W]")
    axes[1, 1].plot(fine_df["Mass_kg"], fine_df["Endurance_min"], lw=2, color="tab:purple")
    axes[1, 1].set(title="Hover endurance (80% DoD)", xlabel="Mass [kg]", ylabel="Endurance [min]")
    for ax in axes.flat:
        ax.grid(True, alpha=0.3)
    fig.suptitle("Payload & thrust-margin study", fontsize=13)
    fig.tight_layout()
    fig.savefig(PLOTS / "payload_thrust_margin.png", dpi=300)
    plt.close(fig)

    # ---- 3. Parameter sensitivity ----------------------------------------
    print("\n" + "=" * 78)
    print("3. +/-10% ONE-AT-A-TIME PARAMETER SENSITIVITY")
    print("=" * 78)
    base_omega, base_rpm, base_t_per = hover_point(M_BASE, kf)
    base_shaft = 4 * km * base_omega**3
    base_elec = 4 * float(p_pred(base_t_per))
    eta_hover = base_shaft / base_elec
    print(f"  Baseline kf={kf:.6e}, km={km:.6e}, hover={base_rpm:.1f} RPM, "
          f"T_max={t_max_total:.2f} N, eta_hover={eta_hover:.4f}")

    sens_rows = []
    cases = [("Baseline", D_PROP, CT), ("D -10%", 0.9 * D_PROP, CT),
             ("D +10%", 1.1 * D_PROP, CT), ("CT -10%", D_PROP, 0.9 * CT),
             ("CT +10%", D_PROP, 1.1 * CT)]
    for name, d, ct in cases:
        k, m_ = kf_km(d, ct, CQ)
        omega, rpm, _ = hover_point(M_BASE, k)
        t_max = 4 * k * OMEGA_MAX**2
        shaft = 4 * m_ * omega**3
        elec = shaft / eta_hover
        sens_rows.append({
            "Case": name, "D_m": d, "CT": ct, "kf": k, "km": m_,
            "Hover_Omega_rad_s": omega, "Hover_RPM": rpm,
            "Max_Thrust_Total_N": t_max, "TWR": t_max / (M_BASE * G),
            "Hover_Shaft_Power_Total_W": shaft, "Hover_Elec_Power_Total_W": elec,
            "Delta_Hover_RPM_pct": (rpm / base_rpm - 1) * 100,
            "Delta_Max_Thrust_pct": (t_max / t_max_total - 1) * 100,
            "Delta_TWR_pct": (t_max / t_max_total - 1) * 100,
            "Delta_Hover_Power_pct": (elec / base_elec - 1) * 100,
            "Notes": ("Larger D raises disk area: lower disk loading -> lower hover RPM "
                      "and power, higher kf/km -> higher max thrust and TWR"
                      if name == "D +10%" else
                      "Smaller D raises disk loading: higher hover RPM and power, "
                      "lower max thrust and TWR" if name == "D -10%" else
                      "CT scales kf (thrust, RPM, TWR) but not km; power changes follow"),
        })
    sens_df = pd.DataFrame(sens_rows)
    sens_df.to_csv(HERE / "parameter_sensitivity.csv", index=False)
    print(sens_df[["Case", "Hover_RPM", "Max_Thrust_Total_N", "TWR",
                   "Hover_Elec_Power_Total_W"]].to_string(index=False))
    print("  Disk-loading rationale: kf ~ D^4 and km ~ D^5, so a larger rotor produces "
          "more thrust per rad/s; hover requires less omega (RPM) and less power, "
          "while the omega_max ceiling yields more static thrust and TWR.")

    fig, ax = plt.subplots(figsize=(9.5, 5.6))
    metrics = ["Delta_Hover_RPM_pct", "Delta_Max_Thrust_pct", "Delta_TWR_pct",
               "Delta_Hover_Power_pct"]
    labels = ["Hover RPM", "Max thrust", "TWR", "Hover power"]
    x = np.arange(len(labels))
    width = 0.16
    for i, (case, _, _) in enumerate(cases[1:]):
        vals = [sens_df.loc[sens_df["Case"] == case, c].iloc[0] for c in metrics]
        ax.bar(x + (i - 1.5) * width, vals, width, label=case)
    ax.axhline(0, color="black", lw=0.8)
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel("Change from baseline [%]")
    ax.set_title("One-at-a-time parameter sensitivity (+/-10%)")
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(PLOTS / "parameter_sensitivity.png", dpi=300)
    plt.close(fig)

    # ---- 4. Four-phase mission energy ------------------------------------
    print("\n" + "=" * 78)
    print("4. FOUR-PHASE MISSION ENERGY PROFILE")
    print("=" * 78)
    print("  Profile from report blueprint Section 5.2 (explicit, not fallback values):")
    for name, dur, mult in MISSION_PHASES:
        print(f"    {name:17s} {dur:6.1f} s @ {mult:.2f}*mg")
    mission = phase_breakdown(M_BASE, kf, p_pred, i_pred)
    total_row = {c: "" for c in mission.columns}
    total_row.update({
        "Phase": "TOTAL", "Duration_s": mission["Duration_s"].sum(),
        "Total_Elec_Power_W": "", "Phase_Energy_Wh": mission["Phase_Energy_Wh"].sum(),
        "Phase_Energy_kJ": mission["Phase_Energy_kJ"].sum(),
        "Cumulative_Energy_Wh": mission["Cumulative_Energy_Wh"].iloc[-1],
        "Cumulative_Ah_Used": mission["Cumulative_Ah_Used"].iloc[-1],
        "End_SoC_pct": mission["End_SoC_pct"].iloc[-1],
        "Notes": "Blueprint 4-phase profile; 80% usable DoD; SoC from 100%",
    })
    mission_out = pd.concat([mission, pd.DataFrame([total_row])], ignore_index=True)
    mission_out.to_csv(HERE / "mission_energy.csv", index=False)
    final_soc = float(mission["End_SoC_pct"].iloc[-1])
    print(mission[["Phase", "Duration_s", "Total_Thrust_N", "Total_Elec_Power_W",
                   "Phase_Energy_Wh", "End_SoC_pct"]].to_string(index=False))
    print(f"  TOTAL mission energy = {mission['Phase_Energy_Wh'].sum():.2f} Wh "
          f"({mission['Phase_Energy_kJ'].sum():.2f} kJ), final SoC = {final_soc:.2f} % "
          f"-> reserve (>=20%): {'PASS' if final_soc >= SOC_RESERVE else 'FAIL'}")

    fig, axes = plt.subplots(2, 2, figsize=(11, 7.4))
    # Build a uniform fine time grid across the whole 670 s mission.
    # Thrust and power are piecewise CONSTANT per phase; cumulative energy
    # ramps LINEARLY and SoC falls LINEARLY between phase boundaries.
    bounds = np.concatenate([[0.0], np.cumsum(mission["Duration_s"].to_numpy())])
    t_fine = np.linspace(0.0, bounds[-1], int(bounds[-1] * 4) + 1)  # 4 Hz
    phase_idx = np.clip(np.searchsorted(bounds, t_fine, side="right") - 1,
                        0, len(mission) - 1)
    cum_wh_bounds = np.concatenate([[0.0], mission["Cumulative_Energy_Wh"].to_numpy()])
    soc_bounds = np.concatenate([[100.0], mission["End_SoC_pct"].to_numpy()])
    traces = {
        "Total_Thrust_N": mission["Total_Thrust_N"].to_numpy()[phase_idx],
        "Total_Elec_Power_W": mission["Total_Elec_Power_W"].to_numpy()[phase_idx],
        "Cumulative_Energy_Wh": np.interp(t_fine, bounds, cum_wh_bounds),
        "End_SoC_pct": np.interp(t_fine, bounds, soc_bounds),
    }
    for ax, col, title, ylabel in zip(
        axes.flat,
        ["Total_Thrust_N", "Total_Elec_Power_W", "Cumulative_Energy_Wh", "End_SoC_pct"],
        ["Total thrust", "Total electrical power", "Cumulative energy", "Battery SoC"],
        ["Thrust [N]", "Power [W]", "Energy [Wh]", "SoC [%]"],
    ):
        ax.plot(t_fine, traces[col], lw=2)
        ax.fill_between(t_fine, traces[col], alpha=0.12)
        for b in bounds:
            ax.axvline(b, color="grey", ls=":", lw=0.7)
        if col == "End_SoC_pct":
            ax.axhline(SOC_RESERVE, color="r", ls="--", lw=1.3, label="20% reserve")
            ax.legend(fontsize=8)
        ax.set(title=title, xlabel="Mission time [s]", ylabel=ylabel)
        ax.grid(True, alpha=0.3)
    fig.suptitle("4-phase mission energy profile", fontsize=13)
    fig.tight_layout()
    fig.savefig(PLOTS / "mission_energy_profile.png", dpi=300)
    plt.close(fig)

    # ---- 5. Battery capacity sensitivity ---------------------------------
    print("\n" + "=" * 78)
    print("5. BATTERY CAPACITY SENSITIVITY (4.0 / 5.0 / 6.0 Ah)")
    print("=" * 78)
    batt_rows = []
    for capacity in (4.0, 5.0, 6.0):
        nominal_wh = V_NOM * capacity
        for mode in ("Fixed-mass", "Mass-adjusted"):
            mass = M_BASE if mode == "Fixed-mass" else M_BASE + (capacity - C_BAT) * 0.10
            k, _ = kf_km()
            omega, _, t_per = hover_point(mass, k)
            hover_elec = 4 * float(p_pred(t_per))
            endurance = DOD_USABLE * nominal_wh / hover_elec * 60
            m_energies = phase_breakdown(mass, k, p_pred, i_pred)
            mission_wh = float(m_energies["Phase_Energy_Wh"].sum())
            # Coulomb counting (same method as mission_energy.csv) so SoC agrees
            ah_used = float(m_energies["Cumulative_Ah_Used"].iloc[-1])
            soc = 100.0 - ah_used / capacity * 100.0
            batt_rows.append({
                "Case_Type": mode, "Capacity_Ah": capacity, "Mass_kg": mass,
                "Nominal_Energy_Wh": nominal_wh,
                "Usable_Energy_Wh_80pct": DOD_USABLE * nominal_wh,
                "Hover_Elec_Power_W": hover_elec, "Hover_Endurance_min": endurance,
                "Mission_Energy_Wh": mission_wh, "Post_Mission_SoC_pct": soc,
                "Reserve_Margin_Satisfied": soc >= SOC_RESERVE,
                "Notes": ("Fixed 2.0 kg isolates capacity effect" if mode == "Fixed-mass"
                          else "0.10 kg per Ah delta vs 5.0 Ah baseline"),
            })
    batt_df = pd.DataFrame(batt_rows)
    batt_df.to_csv(HERE / "battery_sensitivity.csv", index=False)
    print(batt_df[["Case_Type", "Capacity_Ah", "Mass_kg", "Usable_Energy_Wh_80pct",
                   "Hover_Endurance_min", "Mission_Energy_Wh", "Post_Mission_SoC_pct",
                   "Reserve_Margin_Satisfied"]].to_string(index=False))

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.8))
    caps = np.array([4.0, 5.0, 6.0])
    x = np.arange(len(caps))
    w = 0.36
    for j, mode in enumerate(("Fixed-mass", "Mass-adjusted")):
        sub = batt_df[batt_df["Case_Type"] == mode]
        axes[0].bar(x + (j - 0.5) * w, sub["Hover_Endurance_min"], w, label=mode)
        axes[1].bar(x + (j - 0.5) * w, sub["Post_Mission_SoC_pct"], w, label=mode)
    axes[0].set(title="Hover endurance", xlabel="Capacity [Ah]", ylabel="Endurance [min]")
    axes[1].axhline(SOC_RESERVE, color="r", ls="--", lw=1.2, label="20% reserve")
    axes[1].set(title="Post-mission SoC", xlabel="Capacity [Ah]", ylabel="SoC [%]")
    for ax in axes:
        ax.set_xticks(x)
        ax.set_xticklabels([f"{c:.1f}" for c in caps])
        ax.grid(True, axis="y", alpha=0.3)
        ax.legend(fontsize=8)
    fig.suptitle("Battery capacity sensitivity", fontsize=13)
    fig.tight_layout()
    fig.savefig(PLOTS / "battery_capacity_sensitivity.png", dpi=300)
    plt.close(fig)

    # ---- Final validation & summary --------------------------------------
    print("\n" + "=" * 78)
    print("FINAL VALIDATION")
    print("=" * 78)
    expected_csv = ["payload_thrust_margin.csv", "parameter_sensitivity.csv",
                    "mission_energy.csv", "battery_sensitivity.csv"]
    expected_png = ["power_thrust_fit.png", "payload_thrust_margin.png",
                    "parameter_sensitivity.png", "mission_energy_profile.png",
                    "battery_capacity_sensitivity.png"]
    for name in expected_csv:
        p = HERE / name
        assert p.exists() and p.stat().st_size > 0, f"Missing/empty {name}"
        print(f"  OK  {p.relative_to(ROOT)}  ({p.stat().st_size} bytes)")
    for name in expected_png:
        p = PLOTS / name
        assert p.exists() and p.stat().st_size > 0, f"Missing/empty {name}"
        print(f"  OK  {p.relative_to(ROOT)}  ({p.stat().st_size} bytes)")

    print("\nKEY RESULTS")
    print(f"  Power fit R^2          : {p_best['r2']:.6f} (RMSE {p_best['rmse']:.4f} W)")
    print(f"  Hover elec power       : {4*p_hover_1:.2f} W total ({p_hover_1:.2f} W/rotor)")
    print(f"  2 kg hover endurance   : {payload_df['Est_Hover_Endurance_min'].iloc[0]:.2f} min")
    print(f"  Total mission energy   : {mission['Phase_Energy_Wh'].sum():.2f} Wh")
    print(f"  Final mission SoC      : {final_soc:.2f} % (reserve >=20%: "
          f"{'PASS' if final_soc >= SOC_RESERVE else 'FAIL'})")
    base_batt = batt_df[(batt_df["Capacity_Ah"] == 5.0) &
                        (batt_df["Case_Type"] == "Fixed-mass")].iloc[0]
    print(f"  5 Ah baseline endurance: {base_batt['Hover_Endurance_min']:.2f} min, "
          f"post-mission SoC {base_batt['Post_Mission_SoC_pct']:.2f} %")
    print("  Battery finding        : capacity increases endurance; mass-adjusted "
          "case shows diminishing returns because added Ah adds airframe mass.")
    print("=" * 78)


if __name__ == "__main__":
    main()
