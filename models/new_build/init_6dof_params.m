%% models/new_build/init_6dof_params.m
% Step 4 Parameter Initialization for Quadrotor_6DoF.slx (FRD Body / NED Earth)
clear; clc;

%% 1. Vehicle Mass, Geometry & Inertia (Baseline Case 1)
m = 2.00;                           % Total mass [kg] (2.0, 2.5, or 3.0 kg for Test 4)
g = 9.81;                           % Gravity [m/s^2]
l = 0.225;                          % Moment arm (center to rotor) [m]
Ixx = 0.021; Iyy = 0.021; Izz = 0.040;
J = diag([Ixx, Iyy, Izz]);          % Inertia tensor [kg*m^2]
D_drag = diag([0.0, 0.0, 0.0]);     % Linear body-axis drag matrix [N/(m/s)] (set to 0 per Note Sec 3.2)

%% 2. Verified Propulsion Constants (Bridge from ElectroMechanics_Q_modified.slx)
rho = 1.225;                        % Air density [kg/m^3]
D_prop = 0.254;                     % Propeller diameter (10 in) [m]
CT = 0.110;                         % Thrust coefficient [-]
CQ = 0.0090;                        % Torque coefficient [-]

kf = (CT * rho * D_prop^4) / (4 * pi^2); % Lumped thrust constant: 1.4207e-5 [N/(rad/s)^2]
km = (CQ * rho * D_prop^5) / (4 * pi^2); % Lumped torque constant: 2.9525e-7 [N*m/(rad/s)^2]

RPM_max = 9000;
omega_max = RPM_max * 2 * pi / 60;  % Max rotor speed: 942.48 [rad/s]
omega_max_sq = omega_max^2;         % Max squared speed: 8.8826e5 [(rad/s)^2]
omega_hover = sqrt((m * g) / (4 * kf)); % Hover speed: 587.58 rad/s (5611.0 RPM)

%% 3. FRD Body-Frame Effectiveness & Control Allocation (Mixer) Matrices
% Rotor 1: Front [+l, 0, 0] (CCW) -> Pitch nose UP (+tau_theta), Yaw CW (+tau_psi)
% Rotor 2: Left  [0, -l, 0] (CW)  -> Roll right wing DOWN (+tau_phi), Yaw CCW (-tau_psi)
% Rotor 3: Rear  [-l, 0, 0] (CCW) -> Pitch nose DOWN (-tau_theta), Yaw CW (+tau_psi)
% Rotor 4: Right [0, +l, 0] (CW)  -> Roll right wing UP (-tau_phi), Yaw CCW (-tau_psi)
B_eff = [ kf,     kf,     kf,     kf;
           0,   l*kf,      0,  -l*kf;
        l*kf,      0,  -l*kf,      0;
          km,    -km,     km,    -km ];

B_mixer = inv(B_eff);               % 4x4 Inverse Control Allocation Matrix

%% 4. Baseline PD Attitude & Altitude Controller Gains
Kp_z = 18.0;     Kd_z = 10.8;       % Altitude loop gains (wn = 3 rad/s, zeta = 0.9)
Kp_phi = 0.756;  Kd_phi = 0.2142;   % Roll loop gains     (wn = 6 rad/s, zeta = 0.85)
Kp_theta = 0.756;Kd_theta = 0.2142; % Pitch loop gains    (wn = 6 rad/s, zeta = 0.85)
Kp_psi = 0.640;  Kd_psi = 0.2880;   % Yaw loop gains      (wn = 4 rad/s, zeta = 0.9)

%% 5. Default Test Selector (1=Hover, 2=Roll/Pitch Pulse, 3=Yaw, 4=Payload, 5=Closed-Loop PD)
test_case = 1;
disp('init_6dof_params.m loaded successfully. Hover speed = ' + string(omega_hover) + ' rad/s');