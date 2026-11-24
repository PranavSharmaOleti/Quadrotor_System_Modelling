**PROJECT SCOPE & EXECUTION PLAN**

**Quadrotor System Modelling,  
Characterisation and  
Cross-Simulation Assessment**

A bounded engineering investigation using Simulink, PX4 SITL and Python

<table>
<colgroup>
<col style="width: 100%" />
</colgroup>
<thead>
<tr class="header">
<th><p><strong>PROJECT DECISION</strong></p>
<p><strong>Retain both existing Simulink projects, use PX4 as a bounded independent simulation reference, and combine subsystem findings through a system-level performance assessment. Do not force an immediate full-model integration.</strong></p></th>
</tr>
</thead>
<tbody>
</tbody>
</table>

Prepared: 4 November 2026 \| Version: 1.0 \| Status: Proposed scope

*Planning basis: approximately 15–21 hours of focused project work, distributed across the available month*

# Document Control

| **Item**         | **Description**                                                                                                                                                                                                     |
|------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Document purpose | Define the project objective, technical scope, implementation strategy, responsibilities, schedule, deliverables and acceptance criteria.                                                                           |
| Project status   | Proposed. No corrected model, simulation result or validation outcome is claimed by this document.                                                                                                                  |
| Assessment basis | The final technical report is the assessed output. The project is therefore scoped around a coherent, defensible engineering investigation rather than mechanically reproducing every item in the internship guide. |
| Starting assets  | ElectroMechanics_Q and HoverLock repositories, plus a proposed PX4 SITL reference workflow.                                                                                                                         |
| Evidence basis   | The preliminary model descriptions and defect observations in this report are based on the technical audit supplied during project planning. They must be rechecked against the native files before implementation. |
| Time constraint  | Approximately 2–3 days of concentrated effort in total, spread across roughly one month. The working estimate used here is 15–21 focused hours.                                                                     |

# Report Map

- 1\. Executive Summary and Project Decision

- 2\. Project Background and Starting Assets

- 3\. Problem Statement, Aim and Research Questions

- 4\. Scope, Work Packages and Boundaries

- 5\. Technical Methodology and Governing Checks

- 6\. Proposed Software and Data Architecture

- 7\. Cross-Simulation Comparison Method

- 8\. Work Allocation and AI-Agent Operating Model

- 9\. Execution Schedule and Decision Gates

- 10\. Deliverables and Acceptance Criteria

- 11\. Risks, Fallbacks and Scope-Control Rules

- 12\. Proposed Structure of the Final Technical Report

- 13\. Immediate Kick-off Actions

- Appendices: parameter register, test matrix and performance metrics

| **Reading note:** This is a project plan, not a completed technical report. Proposed methods and acceptance criteria are defined here; final values, plots and conclusions must come from executed models and recorded results. |
|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|

# 1. Executive Summary and Project Decision

This project will develop and assess a component-based quadrotor simulation workflow using the two existing Simulink projects—ElectroMechanics_Q and HoverLock—and, subject to an early feasibility test, PX4 Software-in-the-Loop (SITL) as an independent simulation reference. The objective is to obtain defensible engineering conclusions about propulsion behaviour, basic flight dynamics and selected system-level performance effects without attempting to build a complete production-grade UAV digital twin.

The recommended project direction is to retain the original concept: Quadrotor System Modelling, Characterisation and Cross-Simulation Assessment. The project should not be reduced to propulsion modelling alone. However, the original concept must be executed in a controlled way: the propulsion and flight-dynamics models will initially remain separate, and their results will be connected through analytical calculations and matched operating conditions. A tightly coupled, monolithic Simulink model is not required for the primary deliverables.

The project is organised around three technical investigations:

> **1.** Propulsion-system characterisation using ElectroMechanics_Q: correct and document the motor, propeller and battery representation, then quantify thrust, torque, power and energy behaviour over a defined operating range.
>
> **2.** Flight-dynamics model assessment using HoverLock: audit and repair only the essential physical inconsistencies needed to establish basic hover equilibrium and sensible attitude response.
>
> **3.** Cross-simulation assessment using PX4 SITL: if the environment can be launched and logged quickly, run one carefully bounded scenario and compare selected flight-response quantities. PX4 is a simulation reference, not experimental ground truth.

A fourth activity combines the findings at system level. Propulsion outputs will be used to estimate thrust margin and electrical demand for selected flight conditions. Payload and mission-phase effects will be studied using the flight-performance relationships and propulsion data. This system-level assessment does not require the two Simulink files to be merged.

| **Recommended scope decision:** Develop two credible subsystem investigations, attempt one controlled PX4 comparison, and integrate the evidence analytically. Treat full model coupling, advanced control and autonomy as optional extensions—not project requirements. |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|

The available time is limited. The execution plan therefore targets approximately 15–21 hours of focused work across four sessions. PX4 setup is time-boxed to 2–3 hours; basic HoverLock repair is time-boxed to approximately 3–4 hours. If either workstream exceeds its limit, the defined fallback will be used rather than allowing one subsystem to consume the project.

# 2. Project Background and Starting Assets

## 2.1 Existing project repositories

The project starts from two existing repositories supplied for assessment:

<table>
<colgroup>
<col style="width: 33%" />
<col style="width: 33%" />
<col style="width: 33%" />
</colgroup>
<thead>
<tr class="header">
<th><strong>Repository</strong></th>
<th><strong>Role in proposed project</strong></th>
<th><strong>Starting use</strong></th>
</tr>
</thead>
<tbody>
<tr class="odd">
<td>ElectroMechanics_Q<br />
https://github.com/PranavSharmaOleti/ElectroMechanics_Q</td>
<td>Propulsion and electrical subsystem</td>
<td>Primary starting point for the motor–propeller–battery model and propulsion characterisation.</td>
</tr>
<tr class="even">
<td>HoverLock<br />
https://github.com/PranavSharmaOleti/HoverLock</td>
<td>Airframe and flight-dynamics subsystem</td>
<td>Starting point for basic translational/rotational dynamics and closed-loop attitude response, after physical audit and repair.</td>
</tr>
</tbody>
</table>

## 2.2 Preliminary audit: ElectroMechanics_Q

The preliminary audit supplied during planning describes ElectroMechanics_Q as a four-motor propulsion bench model. It contains a table-based battery representation, four motor/propeller branches, aggregate thrust/torque sensing and a state-of-charge output. Two motor branches use a negative gain to represent opposite rotor directions. The current command structure is a constant torque command applied to the motors.

This is a useful foundation because it already contains the main propulsion-system elements and has a relatively coherent, narrow purpose. It is not yet a validated or physically trustworthy model. The audit identified parameters that appear inconsistent or implausible for a small quadrotor. Examples noted in the supplied audit include a 1.5 m propeller diameter, 10 kg propeller mass and 1 kg·m² propeller inertia, alongside motor limits and electrical values that appear mutually inconsistent (including a 1 mN·m torque limit, 3 W power limit and zero winding resistance). These values must be verified in the native model and replaced only through a traceable configuration decision.

The audit also associated the battery block with a Panasonic NCA593446 cell and a nominal capacity of 1.257 Ah. That identification and the pack configuration must be checked against the actual block parameters and a manufacturer source before being treated as the model specification.

## 2.3 Preliminary audit: HoverLock

The preliminary audit describes HoverLock as the more ambitious flight-system model. Its HoverSystem model contains a feedback loop around an airframe, sensor-dynamics block and flight-control system. The airframe includes translational and rotational dynamics, gravity, drag and hard-stop elements, with embedded functions used for rotation-matrix and body-rate/Euler-rate calculations.

The supplied audit identified several issues that prevent the model from being treated as a reliable baseline without repair:

- The airframe mass is reported as 0.07 kg while a root-level force constant is 19.62 N, equivalent to the weight of approximately 2 kg under standard gravity. The intended vehicle mass and applied force therefore require reconciliation.

- The PID blocks reportedly retain default gains (P = 1, I = 1, D = 0), with no clearly established saturation or anti-windup strategy.

- Blocks labelled M1–M4 appear to be copied PID blocks rather than a physically defined four-rotor control-allocation/mixer stage.

- The yaw torque constant is reported as zero, which prevents meaningful yaw-authority assessment.

- The separate FlightControlSystem.slx file is reported as empty, and the Stateflow-related embedded functions appear to be mathematical helpers rather than mission state machines.

- Project notes reportedly acknowledge simulation divergence and propose controller tuning, indicating that the model has not yet demonstrated stable behaviour.

These are preliminary observations from the supplied audit, not a fresh independent inspection of every native block. The first implementation task must confirm them. HoverLock should be repaired only enough to establish physically consistent baseline behaviour; it should not become a full controller-development project.

## 2.4 Role of PX4 SITL

PX4 SITL provides a separate autopilot software stack operating with a simulated vehicle and simulated environment. It can provide repeatable takeoff/hover or attitude-response scenarios and generate ULog telemetry for later analysis. The current PX4 documentation describes SITL as a software flight stack operating with a computer-modelled vehicle and distinguishes simulator configurations such as Gazebo and the lighter-weight SIH path \[1\].

PX4 therefore offers a useful independent simulation reference, but not real-world ground truth. A difference between HoverLock and PX4 may arise from different vehicle parameters, controllers, actuator mappings, coordinate frames or simulator physics. The project must identify which level is being compared and avoid claiming physical validation from simulation-to-simulation agreement.

# 3. Problem Statement, Aim and Research Questions

## 3.1 Problem statement

The existing projects represent different parts of a quadrotor system, but neither should be assumed correct merely because a Simulink model opens or executes. The propulsion model requires parameter and physical-consistency checks. The flight-dynamics model requires reconciliation of mass, force, inertia, rotor geometry, control allocation and sign conventions. In addition, the two models currently do not provide an established common basis for evaluating overall vehicle performance.

The project addresses this gap by defining a coherent reference configuration, checking each subsystem independently, comparing selected flight-response behaviour against a separate autopilot simulation where feasible, and combining verified or explicitly estimated outputs in a system-level performance assessment.

## 3.2 Aim

| **Project aim:** To develop and assess a component-based quadrotor simulation workflow by characterising propulsion and battery behaviour, establishing basic physical consistency in a flight-dynamics model, and comparing selected simulated flight responses against PX4 SITL under clearly defined and suitably matched conditions. |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|

## 3.3 Research questions

| **ID** | **Research question**                                                                                                                                   | **Evidence required**                                                                                        |
|--------|---------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------|
| RQ1    | How do the selected motor, propeller and battery parameters determine thrust, torque, electrical power and energy behaviour across the operating range? | Corrected propulsion model; parameter register; thrust/torque/power curves; independent physical checks.     |
| RQ2    | What corrections are necessary for HoverLock to exhibit physically consistent hover equilibrium and basic attitude response?                            | Mass/inertia/geometry audit; corrected plant; hover force/moment balance; directional response tests.        |
| RQ3    | For one suitably matched scenario, how do selected response variables from the repaired flight-dynamics model compare with a PX4 SITL simulation?       | PX4 ULog; extracted response signals; documented matching assumptions; time-aligned plots and error metrics. |
| RQ4    | How do payload and selected mission operating conditions influence thrust requirements, power demand and estimated energy/endurance?                    | Payload/operating-condition sweeps; analytical calculations; mission energy table; sensitivity analysis.     |

## 3.4 Objectives

- Establish one internally consistent reference quadrotor configuration, with source-traceable values for vehicle mass, rotor geometry, motor, propeller and battery parameters.

- Audit ElectroMechanics_Q and correct the minimum set of parameters and connections required for physically plausible propulsion outputs.

- Verify basic propulsion behaviour using equations, limiting cases, dimensional checks and energy consistency checks.

- Audit HoverLock and correct the minimum set of physical inconsistencies required for a credible hover condition and basic attitude response.

- Attempt one bounded PX4 SITL setup and telemetry extraction workflow; compare a single selected scenario if the setup gate is passed.

- Quantify the effects of payload and selected operating conditions on thrust demand, propulsion power and estimated mission energy.

- Produce a reproducible technical report that distinguishes manufacturer data, derived values, assumptions, simulation results and unvalidated estimates.

# 4. Scope, Work Packages and Boundaries

The project is intentionally structured as a modular investigation. The main work packages are connected through shared parameters and system-level calculations, not by immediate full coupling of the native Simulink models.

## 4.1 Work Package 0 — Baseline audit and reference configuration

Purpose: establish what the existing files actually contain and prevent downstream work from being built on inconsistent parameters.

Activities:

- Open and run both repositories in the available MATLAB/Simulink environment; record MATLAB release, required toolboxes, warnings and run failures.

- Inventory the major blocks, inputs, outputs, solver settings, units and initial conditions in each model.

- Create a single reference-configuration register. Every numerical input must be classified as manufacturer-specified, measured, derived or assumed.

- Resolve the intended mass, number and geometry of rotors, motor/propeller pairing, battery pack arrangement and coordinate conventions.

- Create a change log and retain an untouched copy of each original repository/model.

Output: baseline audit, parameter register, selected configuration and a list of accepted corrections.

## 4.2 Work Package 1 — Propulsion-system modelling and characterisation

Purpose: establish a physically plausible mapping between motor operating condition, propeller output and electrical demand using ElectroMechanics_Q as the primary implementation base.

Activities:

- Inspect the motor-drive, propeller, battery and sensing blocks. Confirm whether their units and parameter definitions correspond to the equations used in the model.

- Choose a consistent motor–propeller–battery configuration using manufacturer data where available. Avoid combining unrelated component specifications simply to populate missing values.

- Correct propeller diameter, mass and inertia; motor torque/speed/power limits; winding resistance; motor inertia; propeller coefficients; and battery electrical parameters as required.

- Confirm rotor direction and reaction-torque sign convention for the four-motor arrangement. Equal-thrust hover should produce near-zero net roll, pitch and yaw moments for a symmetric configuration.

- Run operating-point sweeps over a defined RPM or motor-command range and record thrust, torque, current, voltage, electrical power, shaft power and state of charge where the model supports them.

- Cross-check output magnitudes against independent analytical relationships and identify any parameter ranges where the model becomes implausible.

Core outputs: corrected model; parameter/source register; thrust–RPM, torque–RPM and power–RPM curves; battery voltage/current/energy traces for selected operating points; verification table.

## 4.3 Work Package 2 — Flight-dynamics model assessment

Purpose: determine whether HoverLock can represent basic quadrotor dynamics without attempting a complete autopilot or autonomy implementation.

Activities:

- Reconcile total mass and gravitational force. Confirm whether the 0.07 kg mass and 19.62 N force constant in the preliminary audit are genuine model values or artefacts of the initial inspection.

- Check inertia matrix, rotor arm lengths, rotor positions, spin directions, thrust axes and body/world coordinate conventions.

- Confirm the rotational equations, gravity direction, translational drag and any damping terms.

- Check that the motor mixer/control-allocation logic maps collective thrust and roll/pitch/yaw moments into four feasible rotor commands. Do not treat four duplicated PID blocks as a validated mixer.

- Check that motor thrust and reaction torque signs are consistent with rotor locations and rotation direction. Restore non-zero yaw authority only when the underlying physical model supports it.

- First test the plant at open-loop equilibrium. Only after that should the basic attitude controller be tuned or repaired.

- Add output saturation and anti-windup only as needed for stable and interpretable basic tests; advanced controller design is outside the core scope.

Core outputs: corrected/annotated HoverLock model if feasible; mass/inertia/geometry audit; force and moment balance; basic hover test; basic roll/pitch/yaw direction checks; a documented list of residual limitations.

## 4.4 Work Package 3 — PX4 SITL cross-simulation assessment

Purpose: use PX4 SITL as a repeatable, separate simulation reference for one limited flight-response comparison. The aim is to compare simulated behaviour under documented conditions, not to establish real-world accuracy.

Activities:

- Attempt one supported PX4 SITL configuration and launch a default multicopter scenario. Prefer the simplest environment capable of producing the required data; do not install a full ROS 2 autonomy stack for this project.

- Run one uncomplicated scenario, preferably takeoff to stabilised hover or a small attitude-response test. Save the original ULog and the PX4 parameter set.

- Use pyulog or the supported ULog conversion scripts to inspect and extract relevant topics. The pyulog project documents ULog parsing and conversion to CSV, among other utilities \[2\].

- Identify which signals are actually present in the log: vehicle attitude, angular rates, position/velocity, setpoints and actuator outputs as available.

- Before comparison, reconcile units, coordinate frames, axis signs, timestamp origin, sampling and vehicle configuration.

- Compare only variables with equivalent definitions. Use one scenario and a small, explicit set of metrics.

Core output if the gate is passed: scenario description, configuration snapshot, ULog, extraction script, time-aligned comparison plots and error metrics. If PX4 cannot be established within the time limit, omit the cross-simulation claim and use the fallback described in Section 11.

## 4.5 Work Package 4 — System-level performance assessment

Purpose: combine the separately characterised propulsion and flight-dynamics results to evaluate a small number of system-level questions.

Activities:

- Calculate required total and per-rotor thrust for hover and selected vertical-climb conditions.

- Estimate thrust margin using the characterised maximum or selected operating-point thrust.

- Run payload sweeps to quantify changes in vehicle weight, hover thrust and estimated electrical power.

- Define a simple mission profile using only phases that the available models can support credibly (for example, takeoff/climb, hover and descent).

- Integrate or sum electrical energy over the mission phases and estimate endurance with explicit assumptions about usable battery energy and operating limits.

- Perform sensitivity analysis on uncertain parameters such as thrust coefficient, motor efficiency, battery resistance and usable capacity.

Core outputs: payload-versus-thrust/power results; estimated mission-energy budget; endurance estimates with caveats; sensitivity results; integrated discussion linking subsystem findings.

## 4.6 Explicit exclusions

| **Excluded activity**                                                                          | **Reason for exclusion**                                                                            |
|------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------|
| Full closed-loop integration of HoverLock and ElectroMechanics_Q into one large Simulink model | High integration/debugging risk; not necessary for the research questions or system-level analysis. |
| Advanced LQR, MPC, adaptive or optimal control                                                 | Would create a separate controller-design project and dilute the modelling/characterisation focus.  |
| Autonomous mission planning, Stateflow mission logic, ROS 2 autonomy                           | Not necessary for the planned engineering evidence and too large for the available effort.          |
| Computer vision, YOLO, LiDAR SLAM, perception or obstacle avoidance                            | Outside the chosen propulsion/flight-dynamics investigation.                                        |
| Embedded code generation, SIL/HIL deployment or hardware flight testing                        | Requires additional licences, hardware, setup and validation beyond the project window.             |
| Claims of experimental validation or real-world predictive accuracy                            | No independent physical test data has been established for this project.                            |
| Large numbers of unmotivated plots or cosmetic dashboards                                      | Deliverables must answer a research question, not merely demonstrate software activity.             |

# 5. Technical Methodology and Governing Checks

## 5.1 Reference configuration and data hierarchy

The model should represent one coherent quadrotor configuration. The selected configuration may be a documented existing vehicle or a clearly identified hypothetical research configuration. The report must not imply that the project represents a specific commercial UAV unless the component and airframe specifications support that claim.

Use the following evidence hierarchy when selecting parameters:

> **1.** Manufacturer datasheet or official technical specification for the exact component and operating conditions.
>
> **2.** Independent test data or a reputable technical source describing the same component/configuration.
>
> **3.** A value derived from traceable specifications using a documented equation.
>
> **4.** A modelling assumption selected because the required information is unavailable. The assumption and its sensitivity must be stated.

Every parameter should have a value, unit, source, classification, confidence note and model location. If a parameter cannot be traced, it must remain visibly marked as an assumption rather than silently presented as a measured or manufacturer value.

## 5.2 Propeller and motor relationships

For a propeller operating in a regime where the usual nondimensional coefficients are applicable, thrust and torque may be represented as:

T = C_T · ρ · n² · D⁴

Q = C_Q · ρ · n² · D⁵

P_shaft = 2π · n · Q

where T is thrust, Q is propeller torque, ρ is air density, n is rotational speed in revolutions per second, D is propeller diameter, and C_T and C_Q are thrust and torque coefficients. The coefficients depend on propeller geometry and operating condition; they must be sourced, measured or explicitly assumed. They must not be invented to make the model output look plausible.

Electrical input power is evaluated from P_elec = V·I. Shaft power and electrical input power will differ because of motor and ESC losses. If the model includes efficiency, verify how it is defined and where it is applied so losses are not counted twice.

For a simplified battery equivalent-circuit model, terminal voltage may be represented as V_term = V_OC(SOC,T) − I·R_0(SOC,T). The exact equation must follow the selected battery block and its parameter definitions. Battery energy consumption can be calculated as E = ∫V·I dt. When V is in volts, I in amperes and time in seconds, divide the integral by 3600 to express energy in watt-hours.

A simple coulomb-counting estimate of state of charge is SOC(t) = SOC(0) − \[1/(3600·Q_Ah)\]∫I dt, where Q_Ah is capacity in ampere-hours and discharge current is positive. This simplified relationship does not capture every battery effect; it should be used only when consistent with the model representation.

## 5.3 Hover and vertical-flight checks

For a symmetric four-rotor vehicle in an ideal steady hover, total thrust balances weight:

T_total ≈ m·g; T_hover,rotor ≈ m·g / 4

This is a first-order force-balance check. It assumes level attitude, equal rotor loading and negligible vertical acceleration. The model should also produce approximately zero net roll, pitch and yaw moment in the symmetric hover condition.

For a vertical climb with upward acceleration a_z and negligible or separately modelled vertical drag, the thrust requirement is:

T_total ≈ m·(g + a_z)

For non-negligible drag, the drag force must be added to the force balance with the correct sign. The thrust margin may be reported as T_available,max/(m·g), provided the numerator is a defensible available-thrust value under the same air-density and battery conditions.

An ideal momentum-theory estimate of induced power in hover is:

P_i ≈ T_total^(3/2) / √(2·ρ·A_total)

where A_total is the sum of the four rotor disk areas, assuming non-overlapping disks. This is an ideal lower-bound reference, not the electrical power expected from the actual vehicle. Real power will be higher due to rotor, motor and ESC losses and non-ideal flow.

## 5.4 Mission energy and endurance

The mission should be decomposed into a small number of phases that can be estimated credibly. A suitable initial profile is takeoff/vertical climb, stabilised hover, and descent/landing. Horizontal cruise should be added only if the model includes defensible forward-flight aerodynamics, rotor advance effects and drag. It should not be included as a label attached to a hover-power calculation.

Mission energy is obtained by integrating electrical power over the mission or by summing phase energies: E_mission = Σ(P_avg,phase · t_phase), when average power is a reasonable phase approximation. A first-order endurance estimate is E_usable/P_avg, but the result must account for usable—not merely nominal—battery energy and should be described as an estimate. Voltage sag, current limits, discharge-rate effects, reserves and temperature can change usable energy.

## 5.5 Flight-dynamics verification sequence

| **Test**                 | **Check**                                                       | **Pass condition / interpretation**                                                                                         |
|--------------------------|-----------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------|
| Static hover equilibrium | Sum rotor thrust against weight; check net moment.              | Vertical acceleration approximately zero and net roll/pitch/yaw moments approximately zero in symmetric hover.              |
| Rotor direction and sign | Inspect the sign of thrust and reaction torque for each rotor.  | Rotor spin directions and yaw reaction moments are mutually consistent; no unexplained cancellation or sign reversal.       |
| Roll response            | Apply a small roll command or equivalent differential thrust.   | Vehicle rotates in the expected direction; no unexplained coupling dominates the response.                                  |
| Pitch response           | Apply a small pitch command or equivalent differential thrust.  | Vehicle rotates in the expected direction; sign conventions are documented.                                                 |
| Yaw response             | Apply a small yaw command through differential reaction torque. | Non-zero yaw authority exists only if the rotor torque model supports it; response direction matches the chosen convention. |
| Mass sensitivity         | Increase mass and re-establish hover.                           | Required collective thrust increases consistently with weight.                                                              |

Controller tuning comes after open-loop plant and force/moment checks. Otherwise, a controller may conceal a flawed plant by compensating for wrong signs or incorrect physical constants.

## 5.6 Verification versus validation

Verification asks whether the equations and implementation are being solved consistently: units, sign conventions, force/moment balance, limiting cases and numerical stability. Validation asks whether the model represents the real physical system with adequate accuracy. This project can perform substantial verification and cross-simulation comparison. It cannot claim experimental validation unless independent physical measurements are obtained.

# 6. Proposed Software and Data Architecture

The architecture deliberately separates component modelling, flight dynamics, simulation telemetry and analysis. This reduces coupling and makes it possible to complete meaningful parts even if one environment becomes unavailable.

| **Layer**                    | **Tool / asset**                    | **Primary responsibility**                                          | **Outputs passed forward**                                                   |
|------------------------------|-------------------------------------|---------------------------------------------------------------------|------------------------------------------------------------------------------|
| Component data               | Datasheets, source register         | Establish a coherent motor/propeller/battery/vehicle configuration. | Parameter register with units, source class and confidence.                  |
| Propulsion simulation        | ElectroMechanics_Q / Simulink       | Motor, propeller, battery and thrust/power response.                | Time histories and operating-point tables: thrust, torque, V, I, power, SOC. |
| Flight-dynamics simulation   | HoverLock / Simulink                | Basic translational and rotational response after physical audit.   | Attitude, angular rates, position/velocity, thrust and moment response.      |
| Reference simulation         | PX4 SITL with selected simulator    | One repeatable autopilot-controlled vehicle scenario.               | ULog file, PX4 parameter snapshot and scenario description.                  |
| Data extraction and analysis | Python; pyulog for ULog             | Extract, align, compare, sweep parameters and produce figures.      | CSV data, metrics, plots, uncertainty and sensitivity tables.                |
| System assessment            | Python calculations + model outputs | Connect propulsion capabilities to flight-condition demand.         | Thrust margin, payload trends, mission-energy estimates.                     |
| Reporting                    | Word / Markdown / generated figures | Explain model, method, evidence, findings and limitations.          | Final technical report and supporting files.                                 |

| **Integration principle:** The models remain separate in the first implementation. Their results are combined through a shared reference configuration, common operating conditions and transparent analytical calculations. Full Simulink coupling is optional and must not be required for project completion. |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|

## 6.1 File and version organisation

Recommended project structure:

- 00_admin/ — scope, schedule, decisions and change log.

- 01_original_models/ — untouched copies of the supplied repositories and original model files.

- 02_working_models/ — corrected Simulink models and model-specific notes.

- 03_parameters/ — parameter register, component datasheets and source notes.

- 04_px4/ — PX4 configuration, ULog files, parameter export and scenario notes.

- 05_scripts/ — Python extraction, unit conversion, sweep and plotting scripts.

- 06_results/ — raw outputs, processed data, metrics and final figures.

- 07_report/ — report drafts, final report and presentation material.

Use version control or frequent dated backups. Each native model should have one designated editor at a time. Preserve raw logs and raw simulation outputs; do not overwrite source data with processed files.

# 7. Cross-Simulation Comparison Method

## 7.1 What PX4 can and cannot establish

PX4 SITL can provide a reproducible software-in-the-loop response from a separate autopilot/simulator stack. It can be used to compare the behaviour of two simulated systems for a defined scenario. It cannot establish that either simulation predicts the physical vehicle correctly. Agreement between two simulations may occur because they share assumptions; disagreement may arise from configuration differences rather than a defect in one model.

The PX4 battery indication must not be used as the independent validation reference for the ElectroMechanics_Q battery-energy model unless the battery behaviour in the selected PX4 version and simulator has itself been independently characterised. For this project, the battery model should be checked analytically and through its own electrical consistency tests. PX4 battery telemetry, if recorded, may be shown as simulator telemetry but must not be presented as measured propulsion energy.

## 7.2 Scenario selection

Only one comparison scenario is required. Select the simplest scenario that produces the relevant, analysable outputs. Preferred order:

> **1.** Takeoff followed by stabilised hover, if the initial model states and vehicle parameters can be made comparable.
>
> **2.** A small roll or pitch attitude setpoint response, if a takeoff scenario introduces excessive differences in altitude control, takeoff detection or launch conditions.

Do not attempt a multi-waypoint mission, aggressive manoeuvre set, failure-injection campaign or advanced controller comparison within the core scope.

## 7.3 Matching requirements

Before generating comparison metrics, document the following:

- Vehicle mass and, where possible, inertia and rotor geometry.

- Coordinate systems: world-frame convention (for example, NED versus ENU) and body-frame convention (for example, FRD versus FLU).

- Angle units, angular-rate units, sign conventions and whether angles are Euler angles or quaternion-derived.

- Setpoint profile, initial state, scenario duration and environmental assumptions.

- Controller configuration and actuator mapping. If controllers differ, label the result as a closed-loop system-response comparison rather than a pure plant-model comparison.

- Time bases, sampling rates, missing samples, filtering and any resampling applied.

An input-matched plant comparison is preferred when it is technically practical: apply equivalent rotor or force/moment inputs to the two plant models. If that cannot be achieved quickly, compare closed-loop response under similar reference commands and explicitly state that both controller and plant differences contribute to the result.

## 7.4 Telemetry extraction and metrics

The pyulog package provides Python parsing and command-line utilities for ULog inspection and conversion, including CSV export \[2\]. The extraction script should inspect the actual topics available in the log rather than assume every desired signal was recorded. The original ULog must be preserved.

Candidate comparison signals include attitude, angular rates, local position, velocity, attitude/rate setpoints and actuator outputs, subject to availability. For a scalar signal x(t), a basic root-mean-square error after time and unit alignment is:

RMSE = √\[(1/N) · Σ(x_reference,k − x_model,k)²\]

Additional metrics may include maximum absolute error, steady-state error, overshoot and settling time. For attitude, avoid naive subtraction across angle wrap boundaries. For meaningful three-dimensional orientation comparison, use a quaternion or rotation-error metric, for example θ_error = 2·acos(\|q_ref · q_model\|) after normalising the quaternions and ensuring the same convention.

## 7.5 Interpretation rules

- Report the scenario and parameter differences before presenting error metrics.

- Do not call PX4 the ground truth or describe the comparison as experimental validation.

- Do not compare signals with different units, frames or meanings.

- Do not interpret a lower RMSE as proof that one physical model is more accurate unless an independent reference exists.

- Explain discrepancies in terms of plausible causes: plant parameters, controller, actuator mapping, coordinate convention, solver or simulator physics.

# 8. Work Allocation and AI-Agent Operating Model

The project can use multiple AI coding/research agents to accelerate implementation, but technical accountability must remain centralised. Agents should produce bounded deliverables and should not concurrently edit the same model or script.

| **Role**                | **Primary responsibility**                                                                                                                                      | **Expected deliverables**                                                                      | **Decision authority**                                                  |
|-------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------|
| Project owner / student | Engineering lead: select configuration, approve assumptions, inspect critical behaviour, interpret outputs and own final conclusions.                           | Approved parameter set; test decisions; accepted/rejected model changes; final interpretation. | Final authority on project scope and technical acceptance.              |
| OpenCode                | Repository and implementation agent. Inspect native files, inventory blocks, implement agreed model/script changes and maintain reproducible project structure. | Model inventory; defect list; modified files; change log; automation scripts.                  | May implement approved changes; cannot declare physical validity alone. |
| Codex                   | Independent technical reviewer. Check equations, dimensions, numerical values, model assumptions and Python calculations.                                       | Independent calculation notebooks/scripts; review notes; defect and inconsistency reports.     | Review and challenge; does not silently overwrite implementation.       |
| Antigravity             | Research and documentation agent. Find component sources, extract datasheet values, organise references and assist with technical documentation.                | Source register; datasheet extracts; reference notes; draft documentation.                     | May recommend parameter candidates; project owner approves values.      |
| ChatGPT                 | Technical architect and reviewer. Refine research questions, acceptance tests, comparison methodology, scope decisions and report coherence.                    | Methodology; equations/checks; acceptance criteria; critical review of results and report.     | Advisory and review role; final acceptance remains with project owner.  |

## 8.1 Operating rules

- One owner per file or workstream at a time. An implementation agent changes a model; a separate reviewer checks it.

- Agents must report exact file paths, changed blocks/parameters, units, assumptions and reproducible test steps.

- No agent may replace missing physical data with fabricated values. Missing data must be marked as assumptions and accompanied by sensitivity checks.

- No agent should draft final conclusions before the result set is frozen and reviewed.

- All raw model files, ULogs and raw outputs must be preserved. Processed data must be reproducible from scripts.

- Agent output is a proposal until the project owner has reviewed the engineering basis.

# 9. Execution Schedule and Decision Gates

The work is planned across approximately one month, but the project owner expects to contribute no more than roughly 2–3 days of concentrated effort. The plan therefore assumes approximately 15–21 hours of focused human engineering time, split into four sessions. Agents may handle background implementation and research between sessions, but no background activity is assumed to produce an accepted result without review.

| **Session**                  | **Owner time** | **Main actions**                                                                                                                                                                | **Required exit condition**                                                                                                |
|------------------------------|----------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------|
| 1\. Baseline and feasibility | 3–4 h          | Open and run both models; verify software environment; review parameter and defect inventories; select a coherent reference configuration; attempt one PX4 SITL launch and log. | Known run status for both models; approved initial configuration; PX4 go/no-go decision.                                   |
| 2\. Repair and verification  | 5–7 h          | Review model corrections; test propulsion outputs; establish HoverLock mass/force consistency and hover equilibrium; run basic attitude-direction checks.                       | At least one credible propulsion model and a basic physically sensible flight-dynamics baseline, or a documented fallback. |
| 3\. Results and analysis     | 4–6 h          | Run RPM/operating-point sweeps; run payload and battery/mission sweeps; extract PX4 telemetry if available; inspect and select plots.                                           | Reproducible result dataset and a small set of plots answering the research questions.                                     |
| 4\. Report and defence       | 3–4 h          | Review the report; check source traceability, equations and conclusions; remove unsupported claims; prepare explanations of key results.                                        | Final report with defensible findings, explicit assumptions and reproducible supporting files.                             |

## 9.1 Critical time limits

| **Gate**                | **Time limit**                      | **Decision**                                                                                                                                                                                                                             |
|-------------------------|-------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| PX4 setup gate          | Maximum 2–3 hours                   | If one basic scenario cannot be launched and logged within the limit, stop PX4 troubleshooting. Continue the project without a completed PX4 comparison; do not claim one.                                                               |
| HoverLock repair gate   | Maximum 3–4 hours of focused repair | If basic physical consistency requires major reconstruction, stop repairing the original controller. Use a compact, independently checked flight-dynamics calculation/model for the limited analysis and document HoverLock limitations. |
| Parameter sourcing gate | First session / before final sweeps | If an exact component source cannot be found, use a clearly marked assumption only when the model remains useful and perform sensitivity analysis. Do not block the entire project on perfect data.                                      |
| Report freeze gate      | Before final report assembly        | Freeze the accepted model version, parameter register, raw results and analysis scripts. No new feature should be introduced after this point unless it fixes a material error.                                                          |

## 9.2 Suggested month-level sequence

Week 1: baseline inspection, configuration, PX4 feasibility gate and first model repairs. Week 2: finish propulsion verification and basic flight-dynamics checks. Week 3: generate operating-point, payload and mission-energy results; perform the one PX4 comparison only if feasible. Week 4: freeze results, write and review the final report. The schedule is intentionally not a daily task list; work should be consolidated into the four focused sessions when the project owner is available.

# 10. Deliverables and Acceptance Criteria

## 10.1 Minimum expected deliverables

| **Deliverable**                                  | **Minimum content**                                                                  | **Acceptance condition**                                                                                                |
|--------------------------------------------------|--------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------|
| D1. Baseline audit                               | Inventory of both models, warnings, parameter units, initial conditions and defects. | Original files preserved; defects tied to actual blocks/files; unknowns labelled.                                       |
| D2. Reference parameter register                 | Vehicle, rotor, motor, propeller, ESC and battery configuration with source class.   | Each value has unit, source or explicit assumption label; internally coherent configuration.                            |
| D3. Propulsion model                             | Corrected ElectroMechanics_Q model or equivalent reproducible implementation.        | Produces plausible thrust/torque/electrical outputs and passes basic physical checks.                                   |
| D4. Propulsion verification and characterisation | Operating-point sweeps and independent analytical checks.                            | Plots and tables are reproducible; trends and discrepancies are explained.                                              |
| D5. Flight-dynamics assessment                   | Corrected HoverLock model if feasible, including mass/force and sign checks.         | Hover equilibrium and basic rotational response are demonstrated, or limitations and fallback are documented.           |
| D6. Cross-simulation comparison                  | PX4 scenario, configuration, ULog, extraction and comparison metrics.                | At least one matched/clearly qualified scenario if the setup gate passes. Otherwise omitted without unsupported claims. |
| D7. System-level performance assessment          | Payload/thrust/power and mission-energy estimates.                                   | Assumptions are explicit; results are derived from accepted inputs and operating conditions.                            |
| D8. Final technical report                       | Methodology, models, verification, results, discussion, limitations and references.  | Claims trace to actual evidence; results can be regenerated; project owner can explain the central conclusions.         |

## 10.2 Core acceptance checks

- All component parameters used in final results are sourced, derived or explicitly labelled as assumed.

- Hover force balance is checked against vehicle mass and gravity; equal-thrust symmetric hover produces approximately zero net moment.

- Propeller thrust and torque trends are consistent with the selected coefficients and operating regime.

- Electrical power and integrated energy are internally consistent with voltage and current outputs.

- Increasing payload increases required hover thrust; any non-monotonic result is investigated rather than accepted automatically.

- PX4 comparison signals, where used, have compatible definitions, units, frames and time bases.

- Every final plot can be regenerated from a saved model run or analysis script.

- The report distinguishes simulation, analytical estimate, manufacturer specification, assumption and experimental measurement. No experimental measurement is to be implied unless one is actually obtained.

# 11. Risks, Fallbacks and Scope-Control Rules

| **Risk**                                                                  | **Consequence**                                               | **Mitigation / fallback**                                                                                                                                                 |
|---------------------------------------------------------------------------|---------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| ElectroMechanics_Q contains physically implausible or missing parameters. | A running model may generate meaningless propulsion outputs.  | Create a parameter register; select one coherent configuration; use analytical checks; label uncertain coefficients and perform sensitivity analysis.                     |
| HoverLock is more broken than the preliminary audit suggests.             | Repair effort could consume the project and block results.    | Time-box repair; prioritise mass, inertia, frames, rotor geometry, mixer and hover equilibrium; switch to a compact independently checked dynamics calculation if needed. |
| PX4/Gazebo setup is difficult on the available environment.               | Cross-simulation comparison may not be completed.             | Limit setup to 2–3 hours; try the simplest supported SITL route; retain propulsion and analytical system-level work as core deliverables.                                 |
| PX4 and HoverLock use different vehicle/controller configurations.        | Comparison metrics may be misleading.                         | Match what can be matched; state remaining differences; call it closed-loop response comparison rather than plant validation.                                             |
| Datasheets do not provide required propeller coefficients or motor maps.  | Model performance may depend on uncertain assumptions.        | Use explicit assumptions, derived ranges or sensitivity analysis; do not invent data or claim validation.                                                                 |
| Agents produce conflicting or unreviewed edits.                           | Model versions diverge and defects become difficult to trace. | One file owner at a time; version control/backups; independent reviewer; project-owner approval before merge.                                                             |
| Report drafting starts before results are stable.                         | Narrative may overstate or invent findings.                   | Freeze result data before final report assembly; link every conclusion to a figure, table or calculation.                                                                 |
| Scope expands into autonomy or advanced control.                          | Available time is exhausted without a coherent deliverable.   | Apply explicit exclusions; no new feature after the result-freeze gate unless it fixes a material defect.                                                                 |

## 11.1 PX4 fallback and project-title integrity

If PX4 cannot be made operational within the setup gate, the project must not retain a claim of completed cross-simulation assessment. In that case, change the final report title to “Quadrotor System Modelling, Characterisation and Performance Assessment” and use analytical checks as the independent reference for propulsion and basic flight-performance calculations. The original planned PX4 workflow may be documented briefly as an attempted extension, but it must not be counted as a completed result.

## 11.2 HoverLock fallback

If HoverLock cannot be repaired within the time limit, preserve the audit and explain why it could not serve as a dependable baseline. A compact vertical-flight or rigid-body response model may be used only if it can be implemented and independently checked within the remaining time. Do not replace one large unfinished model with a second large unfinished model.

## 11.3 Scope-control rules

- No full model integration until both subsystems pass their individual checks.

- No advanced control method unless basic plant behaviour is already verified and all primary deliverables are complete.

- No horizontal cruise/endurance claim without a model for forward-flight drag and relevant propeller operating effects.

- No use of default PX4 battery telemetry as a ground-truth measurement of physical battery energy.

- No final conclusion without traceable supporting data.

# 12. Proposed Structure of the Final Technical Report

The final project report should tell an engineering story: why the models were assessed, how the configuration was defined, what was corrected, what was tested, what the results show and what they do not show. The following structure is recommended.

| **Section**                                      | **Content to include**                                                                                                              |
|--------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------|
| 1\. Abstract                                     | Research objective, methods, main quantitative findings and principal limitation. Written after results are frozen.                 |
| 2\. Introduction                                 | Quadrotor system context; need for subsystem modelling and performance assessment; project aim and research questions.              |
| 3\. System definition and configuration          | Vehicle layout, mass properties, rotor arrangement, selected motor/propeller/battery and source-traceable parameter table.          |
| 4\. Existing model audit                         | ElectroMechanics_Q and HoverLock architecture; identified inconsistencies; scope of corrections.                                    |
| 5\. Mathematical modelling                       | Propeller thrust/torque, motor/electrical model, battery model, flight dynamics and control-allocation relationships actually used. |
| 6\. Simulation implementation                    | Simulink model structure, software/tool versions, solver/settings, inputs, initial conditions and modifications.                    |
| 7\. Verification methodology                     | Force/moment balance, units, limits, electrical checks and independent analytical calculations.                                     |
| 8\. Propulsion characterisation results          | Thrust/torque/RPM/power plots, battery traces and interpretation.                                                                   |
| 9\. Flight-dynamics and cross-simulation results | Hover/attitude checks and PX4 comparison, if completed, with configuration caveats and metrics.                                     |
| 10\. System-level performance results            | Payload effects, thrust margin, mission energy and endurance estimates.                                                             |
| 11\. Discussion                                  | Physical interpretation, uncertainty, sensitivity, model discrepancies and limitations.                                             |
| 12\. Conclusion and future work                  | Direct answers to research questions, evidence-supported conclusions and realistic next steps.                                      |
| References and appendices                        | Datasheets, software documentation, parameter register, test cases, additional plots, scripts and model change log.                 |

A good final report should show the engineering effort through the model architecture, correction log, equations, parameter traceability, test matrix, comparison method, plots and interpretation. It should not attempt to create an impression of depth through excessive screenshots or unexplained block diagrams.

# 13. Immediate Kick-off Actions

The next step is not to start building new features. It is to produce a compact evidence package from the current files so that the reference configuration and implementation decisions are made on facts rather than assumptions.

| **Order** | **Assigned role**          | **Immediate task**                                                                                                                                                                     | **Required output**                                                              |
|-----------|----------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------|
| 1         | OpenCode                   | Inspect both native Simulink projects. Extract block inventory, parameters, units, solver settings, warnings and model dependencies. Do not modify the original files.                 | Two concise model inventories and a defect list tied to exact block/model names. |
| 2         | Codex                      | Independently review the key physical equations and reported inconsistencies. Calculate the mass/weight implication of the 19.62 N force constant and identify necessary hover checks. | Technical review with equations, unit checks and prioritised correction list.    |
| 3         | Antigravity                | Find component candidates and datasheets for a coherent motor–propeller–battery configuration. Do not choose a pack or propeller before checking compatibility.                        | Source register with exact URLs, datasheet values, units and missing-data notes. |
| 4         | Project owner              | Run both existing models once in the available environment; review the inventories; choose the vehicle configuration; attempt the PX4 setup gate.                                      | Approved scope/configuration and go/no-go decisions.                             |
| 5         | ChatGPT / technical review | Turn the approved configuration into a test matrix and acceptance checklist; review the first generated plots and report claims.                                                       | Test plan and review notes.                                                      |

| **First-session rule:** Do not allow agents to begin broad model repairs until the baseline inventory and the reference configuration have been reviewed. Preserve the original models and make changes only in working copies. |
|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|

# Appendix A. Parameter Register Template

Every parameter that materially affects the model should be recorded using the following structure. The table below is a template; it is not a list of approved final values.

| **Subsystem**  | **Parameter**                    | **Value** | **Unit** | **Source / method** | **Class**                        | **Confidence / note**                 |
|----------------|----------------------------------|-----------|----------|---------------------|----------------------------------|---------------------------------------|
| Airframe       | Total mass                       |           | kg       |                     | Manufacturer / derived / assumed |                                       |
| Airframe       | Inertia Ixx, Iyy, Izz            |           | kg·m²    |                     | Manufacturer / derived / assumed |                                       |
| Rotor geometry | Arm length / rotor positions     |           | m        |                     | Drawing / measurement / assumed  |                                       |
| Propeller      | Diameter D                       |           | m        |                     | Datasheet / measured / assumed   |                                       |
| Propeller      | Thrust coefficient C_T           |           | —        |                     | Test data / literature / assumed | Operating range must be stated        |
| Propeller      | Torque coefficient C_Q           |           | —        |                     | Test data / literature / assumed | Operating range must be stated        |
| Motor          | Torque constant / speed constant |           | SI units |                     | Datasheet / derived              | Confirm convention and units          |
| Motor          | Winding resistance               |           | Ω        |                     | Datasheet / measured             |                                       |
| Motor          | Current / power limit            |           | A / W    |                     | Datasheet                        |                                       |
| Battery        | Pack voltage / capacity          |           | V / Ah   |                     | Datasheet / pack configuration   | Cell count and series/parallel layout |
| Battery        | Internal resistance              |           | Ω        |                     | Datasheet / measured / assumed   | Temperature and SOC dependence        |
| ESC            | Efficiency / current limit       |           | % / A    |                     | Datasheet / assumed              |                                       |
| Environment    | Air density                      |           | kg/m³    |                     | Atmosphere / assumed             | State temperature and altitude        |

# Appendix B. Minimum Test Matrix

| **Test ID** | **Model**          | **Input / condition**                   | **Record**                                                      | **Purpose**                                |
|-------------|--------------------|-----------------------------------------|-----------------------------------------------------------------|--------------------------------------------|
| P-01        | Propulsion         | Single rotor, low-to-high RPM sweep     | RPM, thrust, torque, current, voltage, power                    | Characterise operating trends and limits.  |
| P-02        | Propulsion         | Selected steady operating point         | Input power, shaft power, torque, voltage/current               | Check electrical/mechanical consistency.   |
| P-03        | Propulsion         | Four equal rotor commands               | Individual and total thrust/moments                             | Check summation and direction conventions. |
| P-04        | Battery            | Defined discharge/load profile          | Current, voltage, SOC, integrated Wh                            | Check voltage/SOC/energy behaviour.        |
| F-01        | Flight dynamics    | Symmetric hover condition               | Vertical acceleration, total thrust, net moment                 | Verify force and moment balance.           |
| F-02        | Flight dynamics    | Small roll/pitch/yaw command            | Attitude, rates, rotor commands                                 | Check axis signs and basic response.       |
| F-03        | Flight dynamics    | Increase vehicle mass                   | Required collective thrust / acceleration                       | Check mass sensitivity.                    |
| X-01        | PX4 + flight model | One takeoff-hover or attitude-step case | Attitude, rates, position/velocity, setpoints, actuator outputs | Limited cross-simulation comparison.       |
| S-01        | System analysis    | Payload sweep                           | Hover thrust, selected power, thrust margin                     | Quantify payload sensitivity.              |
| S-02        | System analysis    | Simple mission phases                   | Phase energy, total energy, estimated endurance                 | Produce a transparent energy budget.       |

# Appendix C. Suggested Result Metrics

| **Metric**                | **Definition / method**                                              | **Use**                                                              |
|---------------------------|----------------------------------------------------------------------|----------------------------------------------------------------------|
| Thrust per rotor          | Rotor thrust at specified RPM or command.                            | Propulsion capability and hover loading.                             |
| Total thrust              | Sum of individual rotor thrusts, with correct axis/sign conventions. | Hover and climb force balance.                                       |
| Thrust margin             | Available total thrust divided by vehicle weight.                    | Indicative lift capability at the stated condition.                  |
| Electrical power          | P = V·I, using consistent voltage/current sign convention.           | Instantaneous propulsion electrical demand.                          |
| Energy                    | Time integral of V·I; report Wh or J with units.                     | Battery demand over a defined operating interval.                    |
| Hover power               | Electrical power at a steady hover operating point.                  | Payload and endurance comparison.                                    |
| Mission energy            | Sum/integral of energy over explicitly defined phases.               | Mission-level performance estimate.                                  |
| Attitude error            | Quaternion/rotation error or wrapped angle error.                    | Cross-simulation orientation comparison.                             |
| Angular-rate RMSE         | Root-mean-square difference after time/unit/frame alignment.         | Compare rotational response.                                         |
| Position/velocity error   | RMSE and peak error after frame alignment.                           | Compare translational response if scenario matching is adequate.     |
| Settling time / overshoot | Measured from a clearly defined command change and tolerance band.   | Response comparison; only meaningful for comparable test conditions. |

# References and Starting Technical Resources

\[1\] PX4 Documentation, “Simulation.” Official PX4 User Guide. https://docs.px4.io/main/en/simulation/ (accessed 4 November 2026). This describes PX4 SITL and simulator configurations; use the version matching the project environment.

\[2\] PX4, “pyulog: Python package and command-line tools for ULog files.” https://github.com/PX4/pyulog (accessed 4 November 2026). The project provides ULog parsing and conversion utilities, including CSV export.

\[3\] PX4 Documentation, “Flight Log Analysis.” https://docs.px4.io/main/en/log/flight_log_analysis (accessed 4 November 2026). Reference for PX4 log-analysis tooling.

\[4\] Project repository supplied for this work: ElectroMechanics_Q. https://github.com/PranavSharmaOleti/ElectroMechanics_Q

\[5\] Project repository supplied for this work: HoverLock. https://github.com/PranavSharmaOleti/HoverLock

# Final Scope Statement

| **Approved intended direction:** The project is a modular quadrotor modelling and assessment study. ElectroMechanics_Q is the primary propulsion-model foundation; HoverLock is the flight-dynamics starting point subject to a strict repair limit; PX4 SITL is a bounded independent simulation reference if its setup gate is passed. The subsystems will be connected through a consistent configuration and system-level calculations, not forced into a full coupled model. The final report will present only executed, reproducible and technically defensible results. |
|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|

*This scope is deliberately ambitious in the quality of its analysis rather than the number of subsystems it attempts to integrate. A smaller set of verified results is preferable to a larger system whose physical assumptions and outputs cannot be defended.*
