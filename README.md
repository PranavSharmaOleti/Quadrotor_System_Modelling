# Quadrotor System Modelling

A traceable, seven-step engineering verification of a 2 kg quadrotor: legacy model audit, Simscape propulsion characterisation, a clean 6-DoF Simulink plant, mission-energy studies, and a PX4 SITL cross-simulation.

> **Note:** The Step 7 comparison is simulation-to-simulation (PX4 Gazebo SITL vs Simulink). No physical flight-test validation is claimed.

## Repository layout

| Folder | Contents |
|---|---|
| `configuration/` | Frozen vehicle parameters, audit register, architecture decision note, component sources |
| `models/original/` | Unmodified legacy `.slx` files (audit evidence) |
| `models/modified/` | Corrected Simscape propulsion model |
| `models/new_build/` | `Quadrotor_6DoF.slx`, `init_6dof_params.m`, Step 5 and Step 7 MATLAB scripts |
| `propulsion_analysis/` | 8-point sweep, verification script/summary, 5 plots |
| `flight_dynamics_6dof/` | Step 5 test outputs (5 tests, plots + Test 4 CSV) |
| `mission_analysis/` | Payload, sensitivity, mission-energy and battery studies + 5 plots |
| `px4_comparison/` | ULog, extracted CSVs, replay/comparison scripts, metrics and 3 plots |
| `report/` | Blueprint, final paper (`.md`, `.pdf`, `.docx`), reproducible build script |

## Key results

- Propulsion errors vs momentum theory: thrust ≤ 0.0073 %, torque ≤ 0.0031 %, shaft power ≤ 0.0032 %
- Bridge constants: `kf = 1.420705e-5`, `km = 2.952483e-7`; hover = 587.6 rad/s (5611 RPM)
- Payload TWR: 2.573 / 2.058 / 1.715 at 2.0 / 2.5 / 3.0 kg
- Mission: 47.72 Wh over 670 s, final SoC 34.15 % (20 % reserve satisfied)
- PX4 vs 6-DoF roll cross-simulation RMSE: 0.629°

## Reproduce the analysis

Requires Python 3 with `numpy`, `pandas`, `matplotlib`, `pyulog` (extraction only). MATLAB/Simulink with Aerospace Blockset is needed to re-run the `.slx` models.

```bash
python3 propulsion_analysis/verification_calculations.py
python3 mission_analysis/run_mission_analysis.py
python3 px4_comparison/scripts/extract_ulog.py          # needs the .ulg log
python3 px4_comparison/scripts/compare_px4_vs_6dof.py   # needs the Simulink replay CSV
python3 report/build_report.py                          # rebuilds PDF + DOCX (needs pandoc + tectonic)
```

## Read the paper

`report/final_report.pdf` (or `.docx`) is the full technical paper. `report/final_report.md` is the source; edit it and rerun `report/build_report.py`.

## Note on binary artifacts

Large `.ulg` logs are gitignored. The extracted CSVs and generated PNGs are committed so the evidence is reproducible without the raw logs.
