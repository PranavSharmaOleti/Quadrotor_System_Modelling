# Component Sources & Engineering Rationale

**Scope.** This note records *why* each subsystem value was selected and how it is
classified. It deliberately avoids inventing commercial part numbers or datasheet
citations: where a value is representative rather than specified, it is labelled as such.
No experimental hardware validation is claimed.

---

## 1. Airframe — 2 kg class, 450 mm quadrotor

- **Classification:** Design Specification (mass, arm length, inertia tensor).
- **Rationale:** A 2.00 kg baseline gives `m·g = 19.62 N`, which cleanly anchors the
  hover condition and matches the payload cases (2.0 / 2.5 / 3.0 kg) used in Test 4.
  An arm length of `l = 0.225 m` corresponds to a 450 mm diagonal wheelbase, a common
  small-UAV frame class. The diagonal inertia tensor `J = diag(0.021, 0.021, 0.040)`
  kg·m² is a symmetric rigid-body approximation suitable for a verification plant.
- **Not claimed:** measured mass properties, CAD-derived inertia, or flight-tested values.

## 2. Propulsion — 920 KV motor + 10-inch propeller pairing

- **Classification:** Representative Component Specification for motor constants and
  inertia; Design Specification for the aerodynamic coefficients used in the model.
- **Rationale:** A 920 KV BLDC on a 4S pack gives a realistic loaded speed ceiling near
  9000 RPM. Pairing it with a 10-inch (0.254 m) rotor using `C_T = 0.110` and
  `C_Q = 0.0090` produces a per-rotor static thrust of ~12.6 N at ~9000 RPM, i.e.
  ~50.5 N total, giving a baseline thrust-to-weight ratio of ~2.57 — appropriate for
  a 2 kg class airframe with payload margin.
- `R_m = 0.085 Ω`, `J_m = 1.5e-5 kg·m²`, and `J_p = 5.0e-5 kg·m²` are representative
  small-motor/propeller values chosen so copper loss and rotor inertia are physically
  present rather than zero or unrealistic.
- **Cross-check:** `K_t = 60/(2π·KV) ≈ 0.0104 N·m/A` is analytically derived from the
  KV rating.
- **Not claimed:** a specific manufacturer, model number, or datasheet curve.

## 3. Powertrain — 4S 5000 mAh LiPo

- **Classification:** Design Specification.
- **Rationale:** A 4S pack gives 14.8 V nominal, which together with the 5000 mAh
  capacity yields ~74 Wh. `R_0 = 0.020 Ω` (5 mΩ per series cell) provides a modest,
  explicit voltage-sag mechanism so the battery-behaviour plot shows physically
  consistent droop under load.
- The 80% usable depth-of-discharge is an explicit mission-study assumption, not a
  manufacturer claim.

## 4. Aerodynamic bridge constants

- **Classification:** Analytically Derived, then Verified from Simscape Sweep.
- **Rationale:** `kf = C_T·rho·D⁴/(4π²)` and `km = C_Q·rho·D⁵/(4π²)` are exact unit
  conversions between the Simscape `n = RPM/60` formulation and the 6-DoF `ω` formulation.
  They are computed from first principles, then independently re-derived from the
  simlog sweep by `propulsion_analysis/verification_calculations.py`.
- **Result:** agreement to ~1e-13 relative; thrust/torque/shaft-power errors < 0.01%.

## 5. Plant — Aerospace Blockset 6DOF (Euler Angles), R2026b

- **Classification:** Design Specification (toolchain choice).
- **Rationale:** The Aerospace Blockset 6DOF block in Metric (MKS) units provides a
  transparent, validated integrator for the 12-state rigid-body system. Using it avoids
  a handwritten Newton–Euler integrator and keeps the frame conventions (FRD body, NED
  earth) explicit at the block boundary. The custom contributions are the force/moment
  computation, the derived 4×4 mixer, and the baseline PD controller.
- **Not claimed:** certification, hardware-in-the-loop validation, or flight data.

## 6. Controller gains

- **Classification:** Tuned Baseline Gain.
- **Rationale:** Gains are chosen from second-order design targets
  (`ωn = 3, 6, 4 rad/s` and `ζ = 0.9, 0.85, 0.9` for altitude/roll-pitch/yaw) and then
  used as the baseline for Test 5. They are simulation-tuned, not hardware-tuned.

---

## Explicit non-claims

- No component is presented as experimentally validated on physical hardware.
- No unverified commercial part numbers or fabricated datasheet citations appear in
  this repository.
- Aerodynamic drag is intentionally zero for the baseline; its omission is a documented
  modelling choice, not a hidden fitting parameter.
