# Quadrotor Step 4 — Architecture Decision Note

**Status:** Final design decision record for the flight-dynamics implementation (Step 4) and the Step 7 cross-simulation convention.

---

## 1. Retirement of `HoverLock.slx` and fresh build of `Quadrotor_6DoF.slx`

The legacy `HoverLock.slx` was preserved unmodified in `models/original/` as audit evidence and formally retired as the simulation baseline. The audit identified:

- **Mass/thrust mismatch:** an airframe mass of `m = 0.07 kg` paired with a hard-coded vertical thrust feed-forward of `19.62 N` (the weight of a `2.00 kg` vehicle at `g = 9.81 m/s²`), a ~28.6× force imbalance.
- **Zero yaw authority:** the yaw reaction-torque constant was set to `km = 0`, so rotor reaction torque `Q_i = km ω_i²` and yaw control were impossible to represent.
- **Non-geometric allocation:** the legacy `M1–M4` routing blocks did not implement a true 4×4 allocator relating roll, pitch and yaw moments to individual rotor speeds.
- **Unverified gains:** controller blocks contained default/uncalibrated PID gains unrelated to the frozen inertia tensor `J = diag(0.021, 0.021, 0.040) kg·m²`.

Repairing these defects would have required reverse-engineering an opaque subsystem hierarchy with unresolved frame and sign conventions. A clean replacement, `Quadrotor_6DoF.slx`, was therefore implemented from first principles, giving complete ownership of every force, moment, frame and state derivative.

## 2. Exact plant architecture

- **Rigid-body plant:** MATLAB/Simulink **Aerospace Blockset `6DOF (Euler Angles)`** block, **Metric (MKS)** units, integrating the 12-state vector `[x, y, z, u, v, w, φ, θ, ψ, p, q, r]`.
- **Frames:** body **FRD** (+X forward, +Y right, +Z down) and flat-Earth **NED** (+X North, +Y East, +Z down). Gravity is applied as `F_g,b = DCM_be · [0, 0, m·g]ᵀ`.
- **Custom MATLAB Function blocks:**
  - `Test_and_PD_Controller` — baseline single-loop PD attitude/altitude controller with test selector (Cases 1–5, plus Case 7 for the replayed setpoint).
  - FRD plus-configuration allocator/mixer built from `B_eff` and `B_mixer = inv(B_eff)`, with rotor speeds saturated to `[0, ω_max²]`.
  - Static quadratic rotor model `T_i = kf ω_i²`, `Q_i = km ω_i²`, with `kf = 1.420705e-5 N/(rad/s)²` and `km = 2.952483e-7 N·m/(rad/s)²`.
- Rotor numbering (plus configuration): 1 Front CCW `[+l, 0, 0]`, 2 Left CW `[0, −l, 0]`, 3 Rear CCW `[−l, 0, 0]`, 4 Right CW `[0, +l, 0]`, `l = 0.225 m`.

## 3. Modeling scope and explicit omissions

- **Zero aerodynamic drag:** `D_drag = diag(0, 0, 0)`. Body drag is intentionally omitted to keep the verification plant analytically traceable; it is a documented limitation, not a fitted term.
- **Instantaneous rotor speed:** no first-order motor/ESC lag is present in the 6-DoF loop; rotor speed responds immediately to the saturated mixer command.
- **No handwritten integrator:** no hand-coded 12-state Newton–Euler integrator is used. Integration is supplied entirely by the Aerospace Blockset 6DOF block.

## 4. Step 7 coordinate-frame rule and reporting scope

`Quadrotor_6DoF.slx` and PX4 SITL both operate in **FRD body / NED local** frames. Therefore **no NED-to-NWU sign flip is applied** to the extracted PX4 telemetry — the legacy conversion in the blueprint belongs to an older NWU formulation and does not apply to the current plant. The absolute PX4 NED heading (~96°) is normalised relative to the window-start value only so the plant can start at `ψ = 0`.

Step 7 is strictly a **simulation-to-simulation cross-comparison**: PX4 Gazebo SITL telemetry versus the Simulink 6-DoF plant under a matched +10° roll command. It evaluates closed-loop simulated behaviour under matched inputs and is **not** physical flight-test validation.
