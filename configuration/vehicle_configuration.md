# Frozen Quadrotor Vehicle Configuration

**Status:** Baseline configuration locked for Steps 1–5. All values are consistent with
`models/new_build/init_6dof_params.m`, `report/quadrotor_master_execution_plan_and_report_blueprint.md`
and the Simscape sweep in `propulsion_analysis/operating_point_sweep.csv`.

**Provenance note:** All component values are *design specifications* or *representative
component specifications*. No experimental hardware validation is claimed anywhere in this
documentation set.

---

## 1. Airframe / Mass Properties

| Parameter | Symbol | Value | Unit |
|---|---|---|---|
| Baseline total mass | `m` | 2.00 | kg |
| Payload test cases (Step 5, Test 4) | — | 2.0, 2.5, 3.0 | kg |
| Gravity | `g` | 9.81 | m/s² |
| Arm length (centre → rotor) | `l` | 0.225 | m |
| Principal inertia tensor (diagonal) | `J` | diag(0.021, 0.021, 0.040) | kg·m² |
| Linear aerodynamic drag matrix | `D_drag` | diag(0, 0, 0) | N/(m/s) |

The 2.00 kg baseline is referenced to `m·g = 19.62 N` and a 450 mm-class diagonal
wheelbase (arm length 0.225 m). The symmetric inertia tensor is a design-level
approximation, not an experimentally identified tensor.

**Intentional omission:** `D_drag` is set to zero for the baseline. Aerodynamic drag is
excluded deliberately to keep the verification plant analytically traceable; it is
documented as a model limitation and is not an overlooked term.

---

## 2. Propulsion

### 2.1 Motor / ESC

| Parameter | Symbol | Value | Unit |
|---|---|---|---|
| Motor class / speed constant | — | 2216-class 920 KV BLDC | — |
| Winding resistance | `R_m` | 0.085 | Ω |
| Torque constant | `K_t` | 0.0104 | N·m/A |
| Motor rotor inertia | `J_m` | 1.5e-5 | kg·m² |
| ESC efficiency | `η_ESC` | 0.95 | — |
| Maximum rotor speed | `RPM_max` | 9000 | RPM |
| Maximum angular speed | `ω_max` | 942.48 | rad/s |
| Maximum squared speed | `ω_max²` | 8.8826e5 | (rad/s)² |

### 2.2 Propeller

| Parameter | Symbol | Value | Unit |
|---|---|---|---|
| Diameter (10 inch) | `D` | 0.254 | m |
| Thrust coefficient | `C_T` | 0.110 | — |
| Torque coefficient | `C_Q` | 0.0090 | — |
| Propeller inertia | `J_p` | 5.0e-5 | kg·m² |

### 2.3 Powertrain (shared battery)

| Parameter | Symbol | Value | Unit |
|---|---|---|---|
| Battery chemistry / configuration | — | 4S LiPo | — |
| Nominal voltage | `V_nom` | 14.8 | V |
| Capacity | `C_bat` | 5.0 (5000 mAh) | Ah |
| Pack internal resistance | `R_0` | 0.020 | Ω |
| Usable depth of discharge (mission study) | DoD | 0.80 | — |

### 2.4 Analytical bridge constants (verified)

Derived from the propeller coefficients and air density, **not** fitted to the sweep:

```
rho  = 1.225 kg/m^3
kf   = (C_T * rho * D^4) / (4*pi^2) = 1.420705e-5 N/(rad/s)^2
km   = (C_Q * rho * D^5) / (4*pi^2) = 2.952483e-7 N*m/(rad/s)^2
```

| Derived quantity | Value | Note |
|---|---|---|
| Hover rotor speed | `ω_hover = 587.58 rad/s` (5611.0 RPM) | `sqrt(m*g / (4*kf))` |
| Total hover thrust | 19.62 N | `m*g`, 4.905 N per rotor |
| Maximum static thrust | 50.48 N | `4*kf*ω_max²` |
| Thrust-to-weight ratio at baseline | 2.57 | from Test 4 summary |
| Thrust margin (baseline) | 30.86 N | `T_max − m*g` |

These constants are independently re-derived by
`propulsion_analysis/verification_calculations.py` and checked against
`init_6dof_params.m` (agreement to ~1e-13 relative).

---

## 3. 6-DoF Architecture & Frames

### 3.1 Plant implementation

- The rigid-body plant is the **Aerospace Blockset 6DOF (Euler Angles)** block operating
  in **Metric (MKS)** units. The 12-state vector is
  `[x, y, z, u, v, w, φ, θ, ψ, p, q, r]`.
- **No handwritten 12-state Newton–Euler integrator is used.** The block supplies the
  translational and rotational integration; the model supplies forces, moments, the
  control allocation and a baseline PD controller.
- Gravity is applied through the body-frame transformation
  `F_gravity_b = DCM_be * [0, 0, m*g]^T` (NED earth gravity expressed in the body frame).
- Attitude kinematics use the ZYX Euler sequence, matching the 6DOF block convention.

### 3.2 Coordinate frames

| Frame | Convention | Axes |
|---|---|---|
| Body (FRD) | Forward–Right–Down | +X forward, +Y right, +Z down |
| Earth (NED) | North–East–Down, flat Earth | +X North, +Y East, +Z down |

PX4 SITL telemetry is logged in NED; the FRD model uses the right-hand body convention
above. Sign conversions between the two are documented in the cross-simulation step
(Step 7) and are not applied here.

### 3.3 Rotor numbering (Quad-+)

| Rotor | Position | Spin | Moment arm |
|---|---|---|---|
| 1 — Front | +X_B | CCW | `[+l, 0, 0]` |
| 2 — Left | −Y_B | CW | `[0, −l, 0]` |
| 3 — Rear | −X_B | CCW | `[−l, 0, 0]` |
| 4 — Right | +Y_B | CW | `[0, +l, 0]` |

### 3.4 Effectiveness and mixer matrices (FRD)

With `U1 = ΣT_i`, `U2 = τ_φ`, `U3 = τ_θ`, `U4 = τ_ψ`:

```
        [ kf      kf      kf      kf  ] [w1²]
B_eff = [ 0      l*kf     0     -l*kf ] [w2²]
        [ l*kf     0    -l*kf     0   ] [w3²]
        [ km     -km      km    -km   ] [w4²]
```

Inverting the effectiveness matrix gives the exact allocation (mixer) matrix used in
`init_6dof_params.m` (`B_mixer = inv(B_eff)`):

```
        [ 1/(4kf)      0        1/(2l*kf)   1/(4km)  ] [U1]
B_mixer=[ 1/(4kf)   1/(2l*kf)    0         -1/(4km)  ] [U2]
        [ 1/(4kf)      0       -1/(2l*kf)   1/(4km)  ] [U3]
        [ 1/(4kf)   -1/(2l*kf)   0         -1/(4km)  ] [U4]
```

Each squared-speed command is saturated to `[0, ω_max²]` before taking the square root:
`ω_i = sqrt(max(0, min(ω_max², ω_i,cmd²)))`.

### 3.5 Baseline PD gains (tuned gains, not hardware-identified)

| Loop | Kp | Kd | Design target |
|---|---|---|---|
| Altitude `z` | 18.0 | 10.8 | ωn = 3 rad/s, ζ = 0.9 |
| Roll `φ` | 0.756 | 0.2142 | ωn = 6 rad/s, ζ = 0.85 |
| Pitch `θ` | 0.756 | 0.2142 | ωn = 6 rad/s, ζ = 0.85 |
| Yaw `ψ` | 0.640 | 0.2880 | ωn = 4 rad/s, ζ = 0.9 |

---

## 4. Resolved legacy defects (audit summary)

| Legacy model | Defect | Resolution |
|---|---|---|
| `ElectroMechanics_Q.slx` | 1.5 m / 10 kg helicopter-scale rotor | Replaced with 0.254 m, 10-inch-class propeller |
| `ElectroMechanics_Q.slx` | 0 Ω winding resistance | Set to `R_m = 0.085 Ω` |
| `ElectroMechanics_Q.slx` | 3.6 V single-cell battery | Replaced with 14.8 V 4S pack, `R_0 = 0.020 Ω` |
| `ElectroMechanics_Q.slx` | Opposing motor spin / thrust cancellation | CW/CCW assignments and signal routing verified |
| `HoverLock.slx` | 0.07 kg airframe paired with 2.0 kg / 19.62 N thrust profile | Model retired; preserved unmodified as audit evidence |
| `HoverLock.slx` | `km = 0` (no yaw authority) | Model retired; replacement uses `km = 2.952483e-7` |

See `configuration/parameter_register_and_audit.csv` for the row-level audit register.
