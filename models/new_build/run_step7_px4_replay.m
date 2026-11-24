%% run_step7_px4_replay.m -- one-click Simulink replay of the PX4 roll step
% NOTE 1: This machine has no MATLAB; the .slx has NOT been modified here.
% NOTE 2: ONE minor edit in Quadrotor_6DoF.slx is required before running:
% NOTE 3: add a From Workspace block (Data = px4_sp_ts, Sample time = 0), wire it
% NOTE 4: to a new sp_ext input on Test_and_PD_Controller, and add case 7 there with
%         phi_ref = sp_ext(1); theta_ref = sp_ext(2); psi_ref = sp_ext(3) - sp_ext0(3);
%         using the SAME PD law as case 5. FRD body / NED earth: NO sign flip.

clear; clc;
run(fullfile(pwd, 'init_6dof_params.m'));   % m = 2.00 kg, kf, km, PD gains, l, omega_max
test_case = 7;
assignin('base', 'test_case', test_case);

%% 1. Read the aligned PX4 setpoint (100 Hz, time-relative to window start)
here = pwd;
root = fullfile(here, '..', '..');
spFile = fullfile(root, 'px4_comparison', 'extracted_csv', 'px4_setpoint_for_simulink.csv');
T = readtable(spFile);
time_s = T.time_s;
phi_sp = T.phi_sp_rad;
theta_sp = T.theta_sp_rad;
psi_sp = T.psi_sp_rad;

% The PX4 log holds an absolute NED heading near 96 deg. The plant starts at
% psi = 0, so replay uses heading RELATIVE to the window-start value. This is
% the only normalisation applied; no FRD/NED sign inversion is performed.
psi_sp_rel = psi_sp - psi_sp(1);

%% 2. Build the Simulink-compatible timeseries in the base workspace
sp_values = [phi_sp, theta_sp, psi_sp_rel];
px4_sp_ts = timeseries(sp_values, time_s, 'Name', 'px4_sp_from_log');
px4_sp_ts.TimeInfo.Units = 'seconds';
assignin('base', 'px4_sp_ts', px4_sp_ts);
assignin('base', 'px4_sp_ts_initial', sp_values(1, :)); % sp_ext0 for case 7
stopTime = num2str(time_s(end));

fprintf('PX4 replay setpoint loaded: %d samples, %.2f s, dt = %.4f s\n', ...
        numel(time_s), time_s(end), time_s(2) - time_s(1));
fprintf('StopTime = %s s | modelName = Quadrotor_6DoF\n', stopTime);

%% 3. Run the pre-edit model
modelName = 'Quadrotor_6DoF';
load_system(modelName);
out = sim(modelName, 'StopTime', stopTime);

%% 4. Export the matched 6-DoF response
t = out.X_e_out.Time(:);
Xe = squeeze(out.X_e_out.Data);
Eu = squeeze(out.Euler_out.Data);
Om = squeeze(out.omega_b_out.Data);
W  = squeeze(out.omega_rotors_out.Data);

if size(Xe, 1) ~= numel(t), Xe = Xe.'; end
if size(Eu, 1) ~= numel(t), Eu = Eu.'; end
if size(Om, 1) ~= numel(t), Om = Om.'; end
if size(W, 1)  ~= numel(t), W  = W.';  end

% FRD body / NED earth: NED z is down-positive => altitude = -z_e
res = table(t, ...
    Eu(:,1), Eu(:,2), Eu(:,3), ...
    rad2deg(Eu(:,1)), rad2deg(Eu(:,2)), rad2deg(Eu(:,3)), ...
    Om(:,1), Om(:,2), Om(:,3), ...
    -Xe(:,3), ...
    W(:,1), W(:,2), W(:,3), W(:,4), ...
    'VariableNames', {'time_s', ...
    'phi_sim_rad','theta_sim_rad','psi_sim_rad', ...
    'phi_sim_deg','theta_sim_deg','psi_sim_deg', ...
    'p_sim_rad_s','q_sim_rad_s','r_sim_rad_s', ...
    'alt_sim_m', ...
    'omega1_rad_s','omega2_rad_s','omega3_rad_s','omega4_rad_s'});

outFile = fullfile(root, 'px4_comparison', 'extracted_csv', 'simulink_6dof_px4_replay.csv');
writetable(res, outFile);
fprintf('Wrote %s (%d rows)\n', outFile, height(res));
fprintf('Next: python3 ../px4_comparison/scripts/compare_px4_vs_6dof.py\n');
