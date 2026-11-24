# Quadrotor UAV Engineering Investigation: Master Execution Plan & Technical Report Blueprint

---

## Part 1: Critical Re-Analysis & Verification (What Changed and Why It Matters)

### 1. Reconciling Work Package Numbering vs. Resume Claims
Between the initial project plan and the architectural update, the work-package labels shifted slightly:
* **Initial Plan:** WP1 = Model Audit, WP2 = Propulsion Characterisation, WP3 = Flight Dynamics & Performance, WP4 = PX4 Cross-Simulation.
* **Architectural Update:** Referenced an earlier scope numbering where WP1 = Propulsion, WP2 = Flight Dynamics (replacing `HoverLock`), and WP3 = PX4 Cross-Simulation.

**Verification Decision:** Structure the final report and evidence repository directly around the **four submitted resume bullets** so there is zero ambiguity during evaluation:
1. **Claim 1 (Audit, Refinement & Control Allocation):** Audit of `ElectroMechanics_Q.slx` and `HoverLock.slx`, parameter corrections to `ElectroMechanics_Q_modified.slx`, formal retirement of `HoverLock.slx`, and implementation of a clean **Quadrotor 6-DoF Rigid-Body Dynamics Model** (`Quadrotor_6DoF.slx`) with geometric control allocation.
2. **Claim 2 (Propulsion System Characterisation):** A 6 to 8 point RPM operating sweep in `ElectroMechanics_Q_modified.slx` paired with four principal characteristic plots and independent analytical verification.
3. **Claim 3 (Flight Dynamics, Payload & Mission Performance):** Execution of the **5 Essential Verification Tests** in `Quadrotor_6DoF.slx` alongside Python studies for 3-case payload sensitivity, thrust margin, motor/propeller sensitivity, and 4-phase mission energy with battery capacity sensitivity.
4. **Claim 4 (PX4 SITL Cross-Simulation):** Telemetry extraction from a PX4 SITL `.ulg` flight log via `pyulog` and quantitative comparison (RMSE, rise time, settling time) against `Quadrotor_6DoF.slx` under a matched attitude setpoint sequence.

---

### 2. Requirements Upgraded From "Conditional" to "Mandatory"
Because `HoverLock.slx` is retired as an unreliable baseline and replaced by a custom 6-DoF model built from first principles, two earlier allowances no longer apply:
* **Yaw Response is Mandatory (Test 3):** In the legacy plan, yaw could be documented as an unimplemented limitation if `HoverLock`'s zero yaw constant could not be repaired. In the new 6-DoF build, rotor reaction torque $Q_i = k_m \omega_i^2$ is explicitly defined, so demonstrating a valid yaw response is mandatory.
* **Closed-Loop PD Attitude Control + Geometric Allocation is Mandatory (Test 5):** The replacement 6-DoF model must include an explicit $4 \times 4$ control-allocation (mixer) matrix derived from rotor geometry, rotor speed saturation limits, and a baseline PD attitude controller that tracks a roll/pitch step without divergence.

---

### 3. Verified Equations & The Exact Model-Coupling Bridge
To connect `ElectroMechanics_Q_modified.slx` and `Quadrotor_6DoF.slx` without real-time co-simulation, both models must share an exact mathematical bridge:

* **In `ElectroMechanics_Q_modified.slx` (Propulsion Sweep & Analytical Verification):**
  Using rotational speed in revolutions per second, $n = \frac{\text{RPM}}{60}$ ($\text{rev/s}$):
  $$T = C_T \rho n^2 D^4, \qquad Q = C_Q \rho n^2 D^5, \qquad P_{\text{shaft}} = Q(2\pi n), \qquad P_{\text{electrical}} = V I$$
  where $\rho = 1.225\text{ kg/m}^3$ is standard air density and $D$ is propeller diameter ($\text{m}$).

* **In `Quadrotor_6DoF.slx` (Rigid-Body Flight Dynamics):**
  Using rotor angular velocity $\omega_i = 2\pi n_i$ ($\text{rad/s}$):
  $$T_i = k_f \omega_i^2, \qquad Q_i = k_m \omega_i^2$$

* **Exact Conversion Bridge (Prevents Unit Mismatch):**
  $$k_f = \frac{C_T \rho D^4}{4\pi^2} \quad \left[\frac{\text{N}}{(\text{rad/s})^2}\right], \qquad k_m = \frac{C_Q \rho D^5}{4\pi^2} \quad \left[\frac{\text{N}\cdot\text{m}}{(\text{rad/s})^2}\right]$$

---

### 4. Three Hidden Engineering Traps to Avoid During Implementation

1. **Trap 1 — Inertial vs. Body Velocity in the Translational Equation:**
   Examine the translational dynamics equation:
   $$m\dot{\mathbf{v}} = R\begin{bmatrix}0\\0\\\sum_{i=1}^4 T_i\end{bmatrix} - \begin{bmatrix}0\\0\\mg\end{bmatrix} - D\mathbf{v}$$
   Because the body thrust vector $[0, 0, \sum T_i]^T$ is rotated by $R$ into the inertial frame and gravity $[0, 0, mg]^T$ is subtracted directly without a rotating-frame Coriolis term ($-\boldsymbol{\omega} \times \mathbf{v}$), the velocity vector $\mathbf{v} = [u, v, w]^T$ in this formulation represents the **inertial-frame velocity** $[\dot{x}, \dot{y}, \dot{z}]^T$. Consequently, position kinematics are simply $\dot{x} = u$, $\dot{y} = v$, $\dot{z} = w$. Do not multiply $\mathbf{v}$ by $R$ a second time when integrating position.
2. **Trap 2 — Inertial Z-Up (NWU/ENU) vs. PX4's Z-Down (NED) Coordinate Convention:**
   In the equation above, gravity acts in the $-Z$ direction and thrust acts in the $+Z_B$ direction, which defines a **Z-Up** coordinate frame (such as North-West-Up, NWU). However, **PX4 SITL logs telemetry in the North-East-Down (NED)** frame where $+Z$ points downward. When comparing PX4 telemetry against your 6-DoF model in Python, invert the signs of $z$, pitch ($\theta$), and yaw ($\psi$) (and their rates $w, q, r$) to align NED with NWU, and document this transformation explicitly.
3. **Trap 3 — Mixer Sign Convention vs. Moments:**
   Ensure the signs in your $4 \times 4$ control allocation matrix exactly match the rotor moment arms ($\mathbf{r}_i \times \mathbf{F}_i$) and CW/CCW reaction torque signs inside the Newton–Euler block. A sign inversion in any row of the mixer turns negative feedback into positive feedback and causes immediate divergence in Test 5.

---

## Part 2: Updated Minimum Viable Engineering Project (The 4 Pillars)

| # | Resume Pillar | Status | Updated Execution Strategy & Required Evidence |
| :--- | :--- | :--- | :--- |
| **1** | **Model Audit, Refinement & Reimplementation** | Mandatory | **Audit both original models.** Refine `ElectroMechanics_Q.slx`. Formally retire `HoverLock.slx` as an unreliable baseline and build a clean **Quadrotor 6-DoF Rigid-Body Dynamics Model** (`Quadrotor_6DoF.slx`).<br>**Evidence:** Defect register for both legacy models, formal rationale for retiring `HoverLock`, before/after parameter tables for `ElectroMechanics_Q`, and the new 6-DoF model architecture. |
| **2** | **Propulsion Characterisation** | Mandatory | Run an 8-point RPM sweep of the corrected motor–propeller–battery system in `ElectroMechanics_Q_modified.slx` and extract thrust/torque constants ($k_f, k_m$) to parameterise the new 6-DoF model.<br>**Evidence:** Thrust, torque, power, current, voltage, and battery-behaviour plots + independent analytical checks at representative RPMs. |
| **3** | **Flight Dynamics & Performance** | Mandatory | Execute the **5 Essential Verification Tests** (hover, roll/pitch, yaw, payload equilibrium, and closed-loop PD attitude control) on `Quadrotor_6DoF.slx`, paired with Python mission-energy and payload sensitivity studies.<br>**Evidence:** 6-DoF state response plots, PD control allocation plots, 3-case payload/thrust-margin table, component sensitivity sweep, and mission-energy/endurance estimates. |
| **4** | **PX4 Cross-Simulation** | Mandatory — Do Not Skip | Execute PX4 SITL, extract telemetry via `pyulog`, feed the matched attitude setpoint sequence into `Quadrotor_6DoF.slx`, and perform a quantitative comparison.<br>**Evidence:** PX4 `.ulg` file, Python extraction script, time-aligned attitude/rate plots, comparison metrics (RMSE, rise/settling time), and discussion of controller/plant differences. |

---

## Part 3: Master Technical Report Blueprint

Use the structure and technical text below as the direct foundation for `report/final_report.docx` and `report/final_report.pdf`.

### Section 1: Executive Summary & Traceability Matrix
This study presents an integrated engineering verification of a quadrotor unmanned aerial vehicle (UAV) spanning propulsion-system characterisation, 6-DoF rigid-body flight dynamics, mission-level energy analysis, and cross-simulation against the PX4 Software-In-The-Loop (SITL) autopilot framework. Rather than coupling legacy Simulink models in real time, a modular verification architecture is implemented:
1. **`ElectroMechanics_Q_modified.slx`** characterises motor, propeller, ESC, and LiPo battery behaviour across steady operating speeds, generating verified aerodynamic coefficients ($k_f, k_m$) and an electrical power curve $P_{\text{elec}}(T)$.
2. **`Quadrotor_6DoF.slx`** replaces the retired legacy flight-dynamics baseline with an explicit 12-state Newton–Euler rigid-body model, geometric $4 \times 4$ control allocation, and a baseline PD attitude controller.
3. **Python Analysis Scripts** integrate the propulsion power curve and vehicle mass properties to evaluate payload sensitivity, thrust margins, mission energy consumption, battery endurance sensitivity, and quantitative trajectory comparisons against PX4 SITL telemetry.

---

### Section 2: Legacy Model Audit & Rationale for Independent 6-DoF Reimplementation

#### 2.1 Audit and Refinement of `ElectroMechanics_Q.slx`
A block-by-block parameter audit of the original `ElectroMechanics_Q.slx` model identified five physical defects that invalidated its baseline outputs:
* **Implausible Propeller Geometry & Mass:** The baseline propeller block specified a diameter of $D = 1.5\text{ m}$ and a mass of $10\text{ kg}$—values representative of a heavy manned helicopter rotor rather than a small quadrotor propeller.
* **Zero Motor Winding Resistance:** Winding resistance was set to $R_m = 0\text{ }\Omega$, eliminating copper heating losses ($I^2 R_m$) and producing non-physical current spikes and artificial efficiency.
* **Contradictory Torque & Power Limits:** Nominal motor torque, maximum torque, and power saturation limits were mutually inconsistent, prematurely clipping shaft output.
* **Unphysical Rotational Inertias:** Motor and propeller inertia parameters ($J_m, J_p$) were mismatched to the physical dimensions of the rotating assembly.
* **Unverified Signal Routing:** Four-motor electrical connections and CW/CCW directional assignments required explicit verification.

All five defects were corrected using a single, datasheet-backed UAV propulsion configuration (Section 3.1) and saved as `ElectroMechanics_Q_modified.slx`.

#### 2.2 Assessment of the Existing Flight-Dynamics Model (`HoverLock.slx`) and Rationale for Independent Reimplementation
Our audit of `HoverLock.slx` revealed fundamental parameter and structural flaws:
* **Severe Mass–Thrust Mismatch:** The vehicle mass was set to $m = 0.07\text{ kg}$, whereas the vertical thrust feed-forward constant was hard-coded to $19.62\text{ N}$ (corresponding to a $2.0\text{ kg}$ airframe under $g = 9.81\text{ m/s}^2$), producing a $28.6\times$ vertical force imbalance.
* **Non-Physical Control Allocation (`M1–M4` Blocks):** The `M1–M4` routing blocks did not implement a true geometric control allocation matrix relating roll, pitch, and yaw moments to individual rotor thrusts.
* **Zero Yaw Reaction-Torque Constant:** The yaw torque coefficient was set to zero ($k_m = 0$), making yaw authority and reaction-torque coupling impossible to simulate.
* **Default Uncalibrated PID Gains:** Controller blocks contained generic default gains unaligned with the airframe inertia tensor.
* **Opaque Coordinate Transformations:** Internal subsystem hierarchy obscured frame definitions and sign conventions, creating a high risk that parameter fixes would uncover deeper structural errors.

| Evaluation Criterion | Repairing `HoverLock.slx` (Rejected) | Building `Quadrotor_6DoF.slx` From Scratch (Selected) |
| :--- | :--- | :--- |
| **Understanding of Architecture** | Requires reverse-engineering opaque blocks | Complete transparency and ownership |
| **Governing Equations** | Must be inferred and audited block-by-block | Explicitly written 12-state Newton–Euler equations |
| **Control Allocation (Mixer)** | Existing `M1–M4` blocks structurally defective | Derived directly from rotor geometry and spin directions |
| **Debugging & Sign Conventions** | High risk of hidden coordinate/frame errors | Predictable, modular MATLAB Function implementation |
| **Engineering Credibility** | Relies on patching an untrusted baseline | Every force, moment, and state derivative is verifiable |

**Engineering Decision:** `HoverLock.slx` was preserved unmodified in `models/original/` as audit evidence and formally retired as the simulation baseline. A clean replacement model, `Quadrotor_6DoF.slx`, was implemented from first principles.

---

### Section 3: Propulsion-System Characterisation & Analytical Verification

#### 3.1 Reference Vehicle & Propulsion Configuration
A consistent $2.0\text{ kg}$ class quadrotor configuration (4S LiPo, $920\text{ KV}$ brushless motor, $10 \times 4.5\text{ in}$ propeller class) is established across all models and scripts:

| Component | Parameter | Symbol | Baseline Value | Unit | Source / Justification |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Vehicle** | Baseline Total Mass | $m_0$ | $2.00$ | $\text{kg}$ | Matched to $mg = 19.62\text{ N}$ reference |
| **Vehicle** | Arm Length (Center to Rotor) | $l$ | $0.225$ | $\text{m}$ | Standard $450\text{ mm}$ diagonal wheelbase |
| **Vehicle** | Principal Inertias | $I_{xx}, I_{yy}, I_{zz}$ | $0.021, 0.021, 0.040$ | $\text{kg}\cdot\text{m}^2$ | Symmetric quadrotor rigid-body tensor |
| **Propeller** | Diameter | $D$ | $0.254$ | $\text{m}$ | $10\text{ inch}$ UAV rotor ($0.254\text{ m}$) |
| **Propeller** | Thrust & Torque Coefficients | $C_T, C_Q$ | $0.110, 0.0090$ | — | Standard static rotor aerodynamic data |
| **Propeller** | Lumped Rotor Constants | $k_f, k_m$ | $1.42 \times 10^{-5}, 2.95 \times 10^{-7}$ | $\text{N}\cdot\text{s}^2, \text{N}\cdot\text{m}\cdot\text{s}^2$ | Derived via $k_f = \frac{C_T\rho D^4}{4\pi^2}, k_m = \frac{C_Q\rho D^5}{4\pi^2}$ |
| **Motor** | Winding Resistance & Torque Constant | $R_m, K_t$ | $0.085, 0.0104$ | $\Omega, \text{N}\cdot\text{m/A}$ | $2216\text{-class } 920\text{ KV}$ BLDC specification |
| **Motor** | Maximum Rotor Speed | $\text{RPM}_{\max}$ | $9000$ ($\omega_{\max} = 942.5$) | $\text{RPM}$ ($\text{rad/s}$) | $4\text{S}$ loaded ceiling |
| **Battery** | Nominal Voltage & Capacity (4S) | $V_{\text{nom}}, C_{\text{bat}}$ | $14.8, 5.0$ | $\text{V}, \text{Ah}$ | $4\text{S } 5000\text{ mAh}$ LiPo pack ($74.0\text{ Wh}$) |
| **Battery** | Pack Internal Resistance | $R_{\text{int}}$ | $0.020$ | $\Omega$ | $5\text{ m}\Omega$ per series cell |
| **ESC** | Solid-State Conversion Efficiency | $\eta_{\text{ESC}}$ | $0.95$ | — | Documented constant approximation |

#### 3.2 Operating-Point Sweep & Analytical Checks
An 8-point steady operating sweep ($2000\text{ to }9000\text{ RPM}$) is executed in `ElectroMechanics_Q_modified.slx`. At each point, the simulated outputs are compared against independent analytical calculations:
1. **Rotational Speed Conversion:** $n = \frac{\text{RPM}}{60}$ ($\text{rev/s}$), $\omega = 2\pi n$ ($\text{rad/s}$).
2. **Propeller Thrust:** $T_{\text{calc}} = C_T \rho n^2 D^4 = k_f \omega^2$.
3. **Propeller Reaction Torque:** $Q_{\text{calc}} = C_Q \rho n^2 D^5 = k_m \omega^2$.
4. **Mechanical Shaft Power:** $P_{\text{shaft}} = Q (2\pi n) = Q \omega$.
5. **Electrical Input Power:** $P_{\text{electrical}} = V_{\text{term}} I$.

**Physical Power Balance Explanation:** Across all operating speeds, $P_{\text{electrical}} > P_{\text{shaft}}$. The difference represents irreversible electromechanical losses:
$$P_{\text{electrical}} - P_{\text{shaft}} = I_m^2 R_m + P_{\text{ESC}} + P_{\text{mech,friction}}$$
where $I_m^2 R_m$ is stator copper Joule heating (which grows quadratically with torque/current) and $P_{\text{ESC}} = (1 - \eta_{\text{ESC}})P_{\text{electrical}}$ captures switching and conduction losses in the electronic speed controller.

---

### Section 4: Mathematical Formulation of the Replacement 6-DoF Model (`Quadrotor_6DoF.slx`)

#### 4.1 Coordinate Convention & 12-State Vector
The model adopts an **Inertial North-West-Up (NWU)** reference frame ($+X$ North, $+Y$ West, $+Z$ Up) and a body-fixed frame ($X_B$ forward, $Y_B$ left, $Z_B$ upward through the rotor plane). The 12-state vector is:
$$\mathbf{x} = \begin{bmatrix} x & y & z & u & v & w & \phi & \theta & \psi & p & q & r \end{bmatrix}^T$$
* **Inertial Position:** $\mathbf{p} = [x, y, z]^T$ ($\text{m}$)
* **Inertial Velocity:** $\mathbf{v} = [u, v, w]^T = [\dot{x}, \dot{y}, \dot{z}]^T$ ($\text{m/s}$)
* **Euler Attitude Angles (ZYX sequence):** Roll $\phi$, Pitch $\theta$, Yaw $\psi$ ($\text{rad}$)
* **Body Angular Rates:** $\boldsymbol{\omega} = [p, q, r]^T$ ($\text{rad/s}$)

#### 4.2 Governing Newton–Euler Equations of Motion
1. **Translational Kinematics:**
   $$\dot{\mathbf{p}} = \mathbf{v} = \begin{bmatrix} u \\ v \\ w \end{bmatrix}$$
2. **Translational Dynamics:**
   $$m\dot{\mathbf{v}} = R(\phi,\theta,\psi)\begin{bmatrix} 0 \\ 0 \\ \sum_{i=1}^4 T_i \end{bmatrix} - \begin{bmatrix} 0 \\ 0 \\ mg \end{bmatrix} - D\mathbf{v}$$
   where $D = \text{diag}(d_x, d_y, d_z)$ is the linear aerodynamic drag matrix and $R(\phi,\theta,\psi)$ is the ZYX body-to-inertial rotation matrix:
   $$R(\phi,\theta,\psi) = \begin{bmatrix} \cos\theta\cos\psi & \sin\phi\sin\theta\cos\psi - \cos\phi\sin\psi & \cos\phi\sin\theta\cos\psi + \sin\phi\sin\psi \\ \cos\theta\sin\psi & \sin\phi\sin\theta\sin\psi + \cos\phi\cos\psi & \cos\phi\sin\theta\sin\psi - \sin\phi\cos\psi \\ -\sin\theta & \sin\phi\cos\theta & \cos\phi\cos\theta \end{bmatrix}$$
3. **Rotational Dynamics:**
   $$J\dot{\boldsymbol{\omega}} = \boldsymbol{\tau} - \boldsymbol{\omega} \times (J\boldsymbol{\omega})$$
   Expanding for a diagonal inertia tensor $J = \text{diag}(I_{xx}, I_{yy}, I_{zz})$ with applied control moments $\boldsymbol{\tau} = [\tau_\phi, \tau_\theta, \tau_\psi]^T = [U_2, U_3, U_4]^T$:
   $$\dot{p} = \frac{U_2 + (I_{yy} - I_{zz})qr}{I_{xx}}, \qquad \dot{q} = \frac{U_3 + (I_{zz} - I_{xx})pr}{I_{yy}}, \qquad \dot{r} = \frac{U_4 + (I_{xx} - I_{yy})pq}{I_{zz}}$$
4. **Euler Angle Kinematics:**
   $$\begin{bmatrix} \dot{\phi} \\ \dot{\theta} \\ \dot{\psi} \end{bmatrix} = \begin{bmatrix} 1 & \sin\phi\tan\theta & \cos\phi\tan\theta \\ 0 & \cos\phi & -\sin\phi \\ 0 & \frac{\sin\phi}{\cos\theta} & \frac{\cos\phi}{\cos\theta} \end{bmatrix} \begin{bmatrix} p \\ q \\ r \end{bmatrix}$$

#### 4.3 Rotor Geometry & Control Allocation Matrix
Using a symmetric **Quad-+** geometry (or **Quad-X** by substituting moment arm $d = l/\sqrt{2}$) with Rotor 1 (Front, $+X_B$, CCW), Rotor 2 (Left, $+Y_B$, CW), Rotor 3 (Rear, $-X_B$, CCW), and Rotor 4 (Right, $-Y_B$, CW):
$$\begin{bmatrix} U_1 \\ U_2 \\ U_3 \\ U_4 \end{bmatrix} = \begin{bmatrix} \sum_{i=1}^4 T_i \\ \tau_\phi \\ \tau_\theta \\ \tau_\psi \end{bmatrix} = \begin{bmatrix} k_f & k_f & k_f & k_f \\ 0 & l k_f & 0 & -l k_f \\ -l k_f & 0 & l k_f & 0 \\ -k_m & k_m & -k_m & k_m \end{bmatrix} \begin{bmatrix} \omega_1^2 \\ \omega_2^2 \\ \omega_3^2 \\ \omega_4^2 \end{bmatrix}$$

Inverting the effectiveness matrix yields the exact **Control Allocation (Mixer) Matrix**:
$$\begin{bmatrix} \omega_{1,\text{cmd}}^2 \\ \omega_{2,\text{cmd}}^2 \\ \omega_{3,\text{cmd}}^2 \\ \omega_{4,\text{cmd}}^2 \end{bmatrix} = \begin{bmatrix} \frac{1}{4k_f} & 0 & -\frac{1}{2lk_f} & -\frac{1}{4k_m} \\ \frac{1}{4k_f} & \frac{1}{2lk_f} & 0 & \frac{1}{4k_m} \\ \frac{1}{4k_f} & 0 & \frac{1}{2lk_f} & -\frac{1}{4k_m} \\ \frac{1}{4k_f} & -\frac{1}{2lk_f} & 0 & \frac{1}{4k_m} \end{bmatrix} \begin{bmatrix} U_1 \\ U_2 \\ U_3 \\ U_4 \end{bmatrix}$$
Each squared speed command is saturated within physical motor limits prior to computing rotor speed:
$$\omega_i = \sqrt{\max\left(0, \, \min\left(\omega_{\max}^2, \, \omega_{i,\text{cmd}}^2\right)\right)}$$

#### 4.4 Baseline PD Attitude & Altitude Controller
For closed-loop verification (Test 5 and PX4 cross-simulation), a baseline PD controller generates the virtual control vector:
$$U_1 = \frac{mg + K_{p,z}(z_{\text{ref}} - z) - K_{d,z} w}{\cos\phi\cos\theta}$$
$$U_2 = K_{p,\phi}(\phi_{\text{ref}} - \phi) - K_{d,\phi} p, \qquad U_3 = K_{p,\theta}(\theta_{\text{ref}} - \theta) - K_{d,\theta} q, \qquad U_4 = K_{p,\psi}(\psi_{\text{ref}} - \psi) - K_{d,\psi} r$$

---

### Section 5: Verification Tests, Mission Performance & PX4 Cross-Simulation

#### 5.1 The 5 Mandatory 6-DoF Verification Tests
| Test # | Test Name | Input Condition | Expected & Verified Physical Behaviour |
| :--- | :--- | :--- | :--- |
| **Test 1** | **Hover Equilibrium** | Open-loop equal thrust $T_i = \frac{mg}{4}$ ($\omega_i = \sqrt{\frac{mg}{4k_f}}$) | Vertical acceleration $\dot{w} = 0$, constant altitude $z(t) = z_0$, zero roll/pitch/yaw angular accelerations ($\dot{p}=\dot{q}=\dot{r}=0$). |
| **Test 2** | **Roll & Pitch Response** | Differential thrust pulse across Left/Right ($\tau_\phi$) and Front/Rear ($\tau_\theta$) rotors | Positive $\tau_\phi$ produces positive roll rate $p$ and angle $\phi$; positive $\tau_\theta$ produces positive pitch rate $q$ and angle $\theta$. |
| **Test 3** | **Yaw Response** | Differential speed between CW ($2,4$) and CCW ($1,3$) rotors with $\sum T_i = mg$ | Pure yaw angular acceleration $\dot{r} = U_4 / I_{zz}$ and yaw angle $\psi(t)$ progression with zero altitude loss. |
| **Test 4** | **Payload Variation** | Three vehicle masses: $m_0 = 2.0\text{ kg}$, $m_1 = 2.5\text{ kg}$, $m_2 = 3.0\text{ kg}$ | Required hover thrust scales linearly ($19.62\text{ N}, 24.53\text{ N}, 29.43\text{ N}$); equilibrium rotor speed increases as $\sqrt{m}$; thrust margin decreases. |
| **Test 5** | **Closed-Loop PD Response** | $10^\circ$ ($0.1745\text{ rad}$) step command in $\phi_{\text{ref}}$ and $\theta_{\text{ref}}$ | Smooth attitude convergence without overshoot instability; individual rotor speeds remain bounded within $[0, \omega_{\max}]$. |

#### 5.2 Payload Sensitivity, Thrust Margin & 4-Phase Mission Energy Analysis
1. **Payload Sensitivity Table (3 Cases):**
   * **Case 1 (Baseline $m_0 = 2.0\text{ kg}$):** $T_{\text{req}} = 19.62\text{ N}$ ($4.91\text{ N/rotor}$). With $T_{\max} = 4 k_f \omega_{\max}^2 \approx 50.46\text{ N}$, Thrust-to-Weight Ratio $\text{TWR} = 2.57$, Thrust Margin $= 30.84\text{ N}$.
   * **Case 2 (Payload +$0.5\text{ kg}$, $m_1 = 2.5\text{ kg}$):** $T_{\text{req}} = 24.53\text{ N}$ ($6.13\text{ N/rotor}$), $\text{TWR} = 2.06$, Thrust Margin $= 25.93\text{ N}$.
   * **Case 3 (Payload +$1.0\text{ kg}$, $m_2 = 3.0\text{ kg}$):** $T_{\text{req}} = 29.43\text{ N}$ ($7.36\text{ N/rotor}$), $\text{TWR} = 1.71$, Thrust Margin $= 21.03\text{ N}$.
2. **4-Phase Mission Profile & Energy Integration:**
   * **Phase 1 (Take-off):** $t \in [0, 5]\text{ s}$, accelerated vertical thrust $T = 1.15\,mg$.
   * **Phase 2 (Climb):** $t \in [5, 35]\text{ s}$, steady climb thrust $T = 1.08\,mg$.
   * **Phase 3 (Hover):** $t \in [35, 635]\text{ s}$, steady hover equilibrium $T = 1.00\,mg$.
   * **Phase 4 (Descent & Landing):** $t \in [635, 670]\text{ s}$, reduced descent thrust $T = 0.92\,mg$.
   * Total electrical energy is integrated via:
     $$E_{\text{mission}} = \int_{0}^{t_f} P_{\text{electrical}}(t)\,dt$$
   * **Model-Based Endurance & Battery Capacity Sensitivity:** Using an $80\%$ usable depth-of-discharge ($E_{\text{usable}} = 0.80 \cdot V_{\text{nom}} \cdot C_{\text{bat}}$), model-based hover/mission endurance is evaluated across $4.0\text{ Ah}$, $5.0\text{ Ah}$, and $6.0\text{ Ah}$ battery configurations (explicitly accounting for the incremental battery mass or holding mass fixed as a sensitivity parameter).

#### 5.3 PX4 SITL Cross-Simulation Methodology & Qualifications
1. **Scenario Execution & Telemetry Extraction:** A repeatable hover-and-attitude-step scenario is executed in PX4 SITL. The resulting `.ulg` log is parsed via `pyulog` to extract `vehicle_attitude` ($\phi, \theta, \psi$), `vehicle_attitude_setpoint` ($\phi_{\text{sp}}, \theta_{\text{sp}}$), `vehicle_angular_velocity` ($p, q, r$), and `actuator_motors`.
2. **Time Alignment & Quantitative Metrics:** The extracted setpoint time series is fed into `Quadrotor_6DoF.slx`. Signals are time-aligned at step initiation $t_0$, and quantitative metrics—**Root Mean Square Error (RMSE)**, **10%–90% Rise Time**, **Peak Response**, and **2% Settling Time**—are computed.
3. **Methodological Qualification:** PX4 SITL employs a cascaded attitude-P / angular-rate-PID architecture with internal rate limiting, whereas `Quadrotor_6DoF.slx` uses a single-loop PD attitude law. Therefore, this cross-simulation evaluates overall closed-loop simulated dynamic behaviour under matched commands rather than isolating bare-airframe plant accuracy or claiming physical flight validation.

---

## Part 4: Step-by-Step Execution Guide (The 7 Major Key Steps)

### Step 1: Freeze the Configuration & Build the Parameter Register (1.5 Hours)
* Create `configuration/parameter_register_and_audit.csv` and `models/new_build/init_6dof_params.m`.
* Lock in your motor, propeller, and battery parameters and compute the exact bridge constants $k_f = \frac{C_T \rho D^4}{4\pi^2}$ and $k_m = \frac{C_Q \rho D^5}{4\pi^2}$.
* Record the legacy defects of both `ElectroMechanics_Q.slx` ($D=1.5\text{ m}, m=10\text{ kg}, R_m=0$) and `HoverLock.slx` ($m=0.07\text{ kg}, F=19.62\text{ N}, k_m=0$) in the defect register.

### Step 2: Modify `ElectroMechanics_Q` and Run the 8-Point RPM Sweep (2.5 Hours)
* Save `models/modified/ElectroMechanics_Q_modified.slx` with corrected winding resistance $R_m$, torque constant $K_t$, rotor inertia, propeller diameter $D$, and battery internal resistance $R_{\text{int}}$.
* Run 8 steady RPM operating points ($2000\text{ to }9000\text{ RPM}$) and export RPM, $n$ ($\text{rev/s}$), Thrust $T$, Torque $Q$, Current $I$, Terminal Voltage $V$, and Electrical Power $P_{\text{elec}}$ to `propulsion_analysis/operating_point_sweep.csv`.

### Step 3: Run the Independent Propulsion Verification Script (1 Hour)
* Execute `propulsion_analysis/verification_calculations.py` to check $T = C_T \rho n^2 D^4$, $Q = C_Q \rho n^2 D^5$, $P_{\text{shaft}} = 2\pi n Q$, and $P_{\text{elec}} = V I$ across all 8 RPM points.
* Generate the 4 required propulsion plots: **Thrust vs RPM**, **Torque vs RPM**, **Power vs Thrust**, and **Battery Behaviour**.

### Step 4: Build `Quadrotor_6DoF.slx` in Simulink (3 Hours)
* Implement `models/new_build/Quadrotor_6DoF.slx` using three clean subsystems feeding a 12-state continuous-time **Integrator** block:
  1. **PD Attitude & Altitude Controller Subsystem** (with an open-loop bypass switch for Tests 1–4).
  2. **Control Allocation & Rotor Saturation Subsystem** ($4 \times 4$ inverse mixer + $[0, \omega_{\max}^2]$ saturation + $T_i = k_f \omega_i^2, Q_i = k_m \omega_i^2$).
  3. **12-State Rigid-Body Dynamics MATLAB Function Block** implementing the exact Newton–Euler equations from Section 4.2.

### Step 5: Execute the 5 Mandatory 6-DoF Verification Tests (2.5 Hours)
* Run and export time-history plots for **Test 1 (Hover Equilibrium)**, **Test 2 (Roll & Pitch Open-Loop Pulse)**, **Test 3 (Yaw Reaction-Torque Response)**, **Test 4 (3-Case Payload Equilibrium)**, and **Test 5 (Closed-Loop PD Step Response)** into `flight_dynamics_6dof/`.

### Step 6: Run Python Payload, Thrust-Margin & Mission-Energy Studies (2 Hours)
* Using the polynomial curve fit $P_{\text{elec}}(T)$ from Step 2, run the Python performance script to output:
  * `mission_analysis/payload_thrust_margin.csv` (Baseline $2.0\text{ kg}$, $2.5\text{ kg}$, $3.0\text{ kg}$).
  * Motor/propeller parameter sensitivity plots ($\pm 10\%$ variation in $D$ or $C_T$).
  * `mission_analysis/mission_energy.csv` and `mission_analysis/battery_sensitivity.csv` for the 4-phase mission profile and 3 battery capacities ($4.0, 5.0, 6.0\text{ Ah}$).

### Step 7: Execute PX4 SITL Scenario, Extract ULog & Run Quantitative Comparison (3 Hours)
* Launch PX4 SITL with Gazebo, execute a hover followed by a roll or pitch step command, and copy the `.ulg` log to `px4_comparison/ulog/`.
* Run `px4_comparison/scripts/extract_ulog.py` (using `pyulog`) to export attitude, attitude setpoints, and angular rates to CSV.
* Feed the extracted setpoint sequence into `Quadrotor_6DoF.slx`, apply the NED-to-NWU sign conversion, and run `compare_px4_vs_6dof.py` to generate overlaid time-aligned plots and compute **RMSE**, **Rise Time**, and **Settling Time**.

---

## Part 5: Required Evidence Folder Structure & Final Checklist

### Evidence Repository Layout
```text
Quadrotor_Project/
│
├── models/
│   ├── original/
│   │   ├── ElectroMechanics_Q.slx
│   │   └── HoverLock.slx                  <-- Preserved unmodified as audit evidence
│   ├── modified/
│   │   └── ElectroMechanics_Q_modified.slx
│   └── new_build/
│       ├── Quadrotor_6DoF.slx             <-- Clean replacement 6-DoF rigid-body model
│       └── init_6dof_params.m             <-- Script loading m, J, l, kf, km, PD gains
│
├── configuration/
│   ├── vehicle_configuration.md
│   ├── parameter_register_and_audit.csv   <-- Includes HoverLock defect & retirement log
│   └── component_sources.md
│
├── propulsion_analysis/
│   ├── operating_point_sweep.csv
│   ├── verification_calculations.py
│   └── plots/
│       ├── thrust_vs_rpm.png
│       ├── torque_vs_rpm.png
│       ├── power_vs_thrust.png
│       └── battery_behaviour.png
│
├── flight_dynamics_6dof/
│   ├── test1_hover_equilibrium/
│   ├── test2_roll_pitch_response/
│   ├── test3_yaw_response/
│   ├── test4_payload_equilibrium/
│   └── test5_pd_control_response/
│
├── px4_comparison/
│   ├── ulog/
│   ├── extracted_csv/
│   ├── scripts/
│   │   ├── extract_ulog.py
│   │   └── compare_px4_vs_6dof.py
│   └── comparison_results/
│       ├── attitude_time_aligned.png
│       └── comparison_metrics.csv
│
├── mission_analysis/
│   ├── payload_thrust_margin.csv
│   ├── mission_energy.csv
│   ├── battery_sensitivity.csv
│   └── plots/
│
└── report/
    ├── final_report.pdf
    └── final_report.docx
```

### Final 9-Point Minimum Acceptance Checklist
- [ ] **1. Model Audit & `HoverLock` Retirement:** Both original models audited; `ElectroMechanics_Q` corrected; `HoverLock` defects documented with a clear engineering rationale for retirement.
- [ ] **2. Physical Configuration:** Coherent vehicle, motor, propeller, and battery parameters documented with datasheet sources or explicit assumptions.
- [ ] **3. Propulsion Operating Sweep:** At least 6–8 RPM points simulated in `ElectroMechanics_Q_modified.slx` with thrust, torque, current, and power outputs + 4 core plots.
- [ ] **4. Analytical Verification:** Independent calculations of $T = C_T \rho n^2 D^4$, $Q = C_Q \rho n^2 D^5$, $P_{\text{shaft}}$, and $P_{\text{elec}}$ across representative RPMs.
- [ ] **5. Clean 6-DoF Model & 5 Essential Tests:** `Quadrotor_6DoF.slx` built with explicit 12-state Newton–Euler equations, geometric mixer, and PD controller; passes Hover, Roll/Pitch, Yaw, Payload equilibrium, and PD step tests.
- [ ] **6. Payload & Thrust-Margin Study:** Baseline plus two payload mass cases comparing required hover thrust against maximum available propulsion thrust.
- [ ] **7. Mission Energy & Battery Study:** 4-phase mission energy integration ($E_{\text{mission}} = \int P_{\text{elec}}(t)\,dt$), model-based endurance estimate, and battery-capacity sensitivity sweep.
- [ ] **8. PX4 SITL Cross-Simulation:** One PX4 SITL `.ulg` extracted via `pyulog`, matched setpoint sequence run in `Quadrotor_6DoF.slx`, time-aligned overlay plots, and $\ge 2$ quantitative metrics (RMSE, rise/settling time).
- [ ] **9. Traceable Technical Report:** Complete documentation of equations, coordinate conventions, `HoverLock` retirement rationale, verification tables, plots, and explicit model limitations.