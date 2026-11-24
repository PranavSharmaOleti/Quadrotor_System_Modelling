%% models/new_build/run_step7_px4_replay.m
% Replays the extracted PX4 SITL attitude setpoint through Quadrotor_6DoF.slx
% and exports simulink_6dof_px4_replay.csv for compare_px4_vs_6dof.py

% 1. Load baseline parameters FIRST (init_6dof_params runs clear; clc;)
init_6dof_params;

% 2. Locate px4_setpoint_for_simulink.csv in the organised repo layout
scriptDir = fileparts(mfilename('fullpath'));
spFile = fullfile(scriptDir, '..', '..', 'px4_comparison', 'extracted_csv', ...
                  'px4_setpoint_for_simulink.csv');

if ~exist(spFile, 'file')
    error('Could not find px4_setpoint_for_simulink.csv at:\n  %s', spFile);
end

fprintf('Loaded PX4 setpoint CSV: %s\n', spFile);
spTbl = readtable(spFile);

% Extract time and [phi_sp, theta_sp, psi_sp] in radians
t_sp   = spTbl{:, 1};
sp_mat = [spTbl{:, 2}, spTbl{:, 3}, spTbl{:, 4}];
sp_mat(:, 3) = atan2(sin(sp_mat(:, 3) - sp_mat(1, 3)), cos(sp_mat(:, 3) - sp_mat(1, 3)));

% Put timeseries + initial setpoint vector into Base Workspace for Simulink
px4_sp_ts = timeseries(sp_mat, t_sp);
sp_ext0   = sp_mat(1, :).';
m         = 2.00;
test_case = 7;

assignin('base', 'px4_sp_ts', px4_sp_ts);
assignin('base', 'sp_ext0',   sp_ext0);
assignin('base', 'm',         m);
assignin('base', 'test_case', test_case);

modelName = 'Quadrotor_6DoF';   % organised repo filename (capital F)


fprintf('Running %s for %.2f s (test_case = 7)...\n', modelName, t_sp(end));
load_system(modelName);
out7 = sim(modelName, 'StopTime', num2str(t_sp(end), '%.6f'));

% 4. Extract logged signals and export matched CSV next to spFile
[t_sim, Xe] = extract_ts(out7.X_e_out);
[~,     Eu] = extract_ts(out7.Euler_out);
[~,     Om] = extract_ts(out7.omega_b_out);
[~,     W]  = extract_ts(out7.omega_rotors_out);

outTbl = table( ...
    t_sim, ...
    Eu(:,1), Eu(:,2), Eu(:,3), ...
    rad2deg(Eu(:,1)), rad2deg(Eu(:,2)), rad2deg(Eu(:,3)), ...
    Om(:,1), Om(:,2), Om(:,3), ...
    -Xe(:,3), ...
    W(:,1), W(:,2), W(:,3), W(:,4), ...
    'VariableNames', { ...
        'time_s', ...
        'phi_sim_rad', 'theta_sim_rad', 'psi_sim_rad', ...
        'phi_sim_deg', 'theta_sim_deg', 'psi_sim_deg', ...
        'p_sim_rad_s', 'q_sim_rad_s', 'r_sim_rad_s', ...
        'alt_sim_m', ...
        'omega1_rad_s', 'omega2_rad_s', 'omega3_rad_s', 'omega4_rad_s'});

outFile = fullfile(fileparts(spFile), 'simulink_6dof_px4_replay.csv');
writetable(outTbl, outFile);
fprintf('Saved 6-DoF replay CSV -> %s (%d rows)\n', outFile, height(outTbl));

% Quick sanity plot
figure('Color', 'w');
plot(t_sp, rad2deg(sp_mat(:,1)), 'k--', 'LineWidth', 1.4); hold on;
plot(t_sim, rad2deg(Eu(:,1)), 'b', 'LineWidth', 1.6); grid on;
xlabel('Time (s)'); ylabel('Roll Angle \phi (deg)');
title('Step 7 Sanity Check: PX4 Roll Setpoint vs Simulink 6-DoF');
legend('PX4 Setpoint', 'Simulink 6-DoF', 'Location', 'best');

function [t, Y] = extract_ts(ts)
    t = ts.Time(:);
    raw = squeeze(ts.Data);
    if size(raw, 1) ~= numel(t) && size(raw, 2) == numel(t)
        Y = raw.';
    else
        Y = raw;
    end
    end