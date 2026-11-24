%% models/new_build/run_step5_tests.m
% Executes all 5 mandatory Step 5 verification tests on Quadrotor_6DoF.slx
% and saves plots + summary CSV into ../../flight_dynamics_6dof/

clear; clc; close all;

% 1. Load baseline parameters into base workspace
init_6dof_params;
modelName = 'Quadrotor_6DoF';
load_system(modelName);
    
% Create exact output directories from Master Blueprint (Part 5)
baseDir = fullfile('..', '..', 'flight_dynamics_6dof');
dirs = { ...
    fullfile(baseDir, 'test1_hover_equilibrium'), ...
    fullfile(baseDir, 'test2_roll_pitch_response'), ...
    fullfile(baseDir, 'test3_yaw_response'), ...
    fullfile(baseDir, 'test4_payload_equilibrium'), ...
    fullfile(baseDir, 'test5_pd_control_response') };
for k = 1:numel(dirs)
    if ~exist(dirs{k}, 'dir'), mkdir(dirs{k}); end
end

%% ========================================================================
%  TEST 1: Hover Equilibrium (m = 2.0 kg, 10 s)
%  ========================================================================
fprintf('Running Test 1: Hover Equilibrium...\n');
m = 2.00; test_case = 1;
assignin('base', 'm', m); assignin('base', 'test_case', test_case);
out1 = sim(modelName, 'StopTime', '10');

[t1, Xe1] = extract_ts(out1.X_e_out);
[~,  Eu1] = extract_ts(out1.Euler_out);
[~,  W1]  = extract_ts(out1.omega_rotors_out);

fig1 = figure('Color','w','Position',[100 100 900 700]);
subplot(3,1,1);
plot(t1, -Xe1(:,3), 'b', 'LineWidth', 1.6); grid on;
ylabel('Altitude -Z_e (m)'); title('Test 1: Open-Loop Hover Equilibrium (m = 2.0 kg)');
ylim([9.5 10.5]);

subplot(3,1,2);
plot(t1, rad2deg(Eu1), 'LineWidth', 1.5); grid on;
ylabel('Euler Angles (deg)'); legend('\phi (Roll)', '\theta (Pitch)', '\psi (Yaw)');
ylim([-1 1]);

subplot(3,1,3);
plot(t1, W1, 'LineWidth', 1.5); grid on;
xlabel('Time (s)'); ylabel('Rotor Speed (rad/s)');
legend('\omega_1', '\omega_2', '\omega_3', '\omega_4');
ylim([550 620]);

exportgraphics(fig1, fullfile(dirs{1}, 'test1_hover_equilibrium.png'), 'Resolution', 300);

%% ========================================================================
%  TEST 2: Roll & Pitch Response (Separate Clean Pulses, 2.0 s each)
%  ========================================================================
fprintf('Running Test 2: Roll & Pitch Open-Loop Pulse Response...\n');
m = 2.00; assignin('base', 'm', m);

% 2a: Pure Roll Pulse
assignin('base', 'test_case', 21);
out2a = sim(modelName, 'StopTime', '2.0');
[t2a, Eu2a] = extract_ts(out2a.Euler_out);
[~,   Om2a] = extract_ts(out2a.omega_b_out);

% 2b: Pure Pitch Pulse
assignin('base', 'test_case', 22);
out2b = sim(modelName, 'StopTime', '2.0');
[t2b, Eu2b] = extract_ts(out2b.Euler_out);
[~,   Om2b] = extract_ts(out2b.omega_b_out);

fig2 = figure('Color','w','Position',[100 100 950 650]);
subplot(2,2,1);
plot(t2a, Om2a(:,1), 'b', 'LineWidth', 1.6); hold on;
plot(t2a, Om2a(:,2), 'r--', 'LineWidth', 1.2); grid on;
ylabel('Body Rate (rad/s)'); title('Test 2a: Roll Pulse (\tau_\phi = +0.01 N\cdotm)');
legend('p (Roll rate)', 'q (Pitch rate)', 'Location', 'northwest');

subplot(2,2,3);
plot(t2a, rad2deg(Eu2a(:,1)), 'b', 'LineWidth', 1.6); hold on;
plot(t2a, rad2deg(Eu2a(:,2)), 'r--', 'LineWidth', 1.2); grid on;
xlabel('Time (s)'); ylabel('Angle (deg)');
legend('\phi (Roll)', '\theta (Pitch)', 'Location', 'northwest');

subplot(2,2,2);
plot(t2b, Om2b(:,2), 'r', 'LineWidth', 1.6); hold on;
plot(t2b, Om2b(:,1), 'b--', 'LineWidth', 1.2); grid on;
ylabel('Body Rate (rad/s)'); title('Test 2b: Pitch Pulse (\tau_\theta = +0.01 N\cdotm)');
legend('q (Pitch rate)', 'p (Roll rate)', 'Location', 'northwest');

subplot(2,2,4);
plot(t2b, rad2deg(Eu2b(:,2)), 'r', 'LineWidth', 1.6); hold on;
plot(t2b, rad2deg(Eu2b(:,1)), 'b--', 'LineWidth', 1.2); grid on;
xlabel('Time (s)'); ylabel('Angle (deg)');
legend('\theta (Pitch)', '\phi (Roll)', 'Location', 'northwest');

exportgraphics(fig2, fullfile(dirs{2}, 'test2_roll_pitch_response.png'), 'Resolution', 300);

%% ========================================================================
%  TEST 3: Pure Yaw Reaction-Torque Response (5.0 s)
%  ========================================================================
fprintf('Running Test 3: Pure Yaw Response...\n');
m = 2.00; assignin('base', 'm', m); assignin('base', 'test_case', 3);
out3 = sim(modelName, 'StopTime', '5.0');

[t3, Xe3] = extract_ts(out3.X_e_out);
[~,  Eu3] = extract_ts(out3.Euler_out);
[~,  Om3] = extract_ts(out3.omega_b_out);
[~,  W3]  = extract_ts(out3.omega_rotors_out);

fig3 = figure('Color','w','Position',[100 100 900 750]);
subplot(3,1,1);
plot(t3, Om3(:,3), 'k', 'LineWidth', 1.6); hold on;
plot(t3, Om3(:,1:2), '--', 'LineWidth', 1.1); grid on;
ylabel('Angular Rate (rad/s)'); title('Test 3: Open-Loop Yaw Reaction-Torque Pulse (\tau_\psi = +0.02 N\cdotm)');
legend('r (Yaw rate)', 'p', 'q', 'Location', 'northwest');

subplot(3,1,2);
yyaxis left;  plot(t3, rad2deg(unwrap(Eu3(:,3))), 'b', 'LineWidth', 1.6); ylabel('Yaw Angle \psi (deg)');
yyaxis right; plot(t3, -Xe3(:,3), 'r--', 'LineWidth', 1.5); ylabel('Altitude -Z_e (m)'); ylim([9.5 10.5]);
grid on; legend('\psi (Yaw)', 'Altitude -Z_e', 'Location', 'northwest');

subplot(3,1,3);
plot(t3, W3, 'LineWidth', 1.5); grid on;
xlabel('Time (s)'); ylabel('Rotor Speed (rad/s)');
legend('\omega_1 (CCW)', '\omega_2 (CW)', '\omega_3 (CCW)', '\omega_4 (CW)', 'Location', 'east');

exportgraphics(fig3, fullfile(dirs{3}, 'test3_yaw_response.png'), 'Resolution', 300);

%% ========================================================================
%  TEST 4: Payload Variation Equilibrium (2.0, 2.5, 3.0 kg)
%  Note: Payload assumed centrally mounted at CoM (baseline J retained;
%  p=q=r=0 in vertical hover equilibrium).
%  ========================================================================
fprintf('Running Test 4: 3-Case Payload Equilibrium...\n');
masses = [2.0, 2.5, 3.0];
T_max = 4 * kf * omega_max^2;
summary = zeros(numel(masses), 6);

fig4 = figure('Color','w','Position',[100 100 900 600]);
colors = {'b', 'g', 'r'};

for i = 1:numel(masses)
    m = masses(i);
    assignin('base', 'm', m); assignin('base', 'test_case', 4);
    out4 = sim(modelName, 'StopTime', '5.0');
    [t4, Xe4] = extract_ts(out4.X_e_out);
    [~,  W4]  = extract_ts(out4.omega_rotors_out);

    w_eq   = mean(W4(end, :));
    rpm_eq = w_eq * 60 / (2*pi);
    T_req  = m * g;
    twr    = T_max / T_req;
    margin = T_max - T_req;
    summary(i, :) = [m, T_req, w_eq, rpm_eq, twr, margin];

    subplot(2,1,1); hold on; grid on;
    plot(t4, -Xe4(:,3), 'Color', colors{i}, 'LineWidth', 1.5, 'DisplayName', sprintf('m = %.1f kg', m));
    ylabel('Altitude -Z_e (m)'); ylim([9.5 10.5]); title('Test 4: 3-Case Payload Hover Equilibrium');

    subplot(2,1,2); hold on; grid on;
    plot(t4, W4(:,1), 'Color', colors{i}, 'LineWidth', 1.6, 'DisplayName', sprintf('m = %.1f kg (%.1f rad/s)', m, w_eq));
    xlabel('Time (s)'); ylabel('Hover Rotor Speed (rad/s)');
end
subplot(2,1,1); legend('Location', 'best');
subplot(2,1,2); yline(omega_max, 'k--', 'LineWidth', 1.2, 'DisplayName', '\omega_{max} (942.5 rad/s)');
ylim([500 980]); legend('Location', 'best');

exportgraphics(fig4, fullfile(dirs{4}, 'test4_payload_equilibrium.png'), 'Resolution', 300);

T_table = array2table(summary, 'VariableNames', ...
    {'Mass_kg', 'Hover_Thrust_N', 'Hover_Omega_rad_s', 'Hover_RPM', 'TWR', 'Thrust_Margin_N'});
writetable(T_table, fullfile(dirs{4}, 'test4_payload_summary.csv'));
disp(T_table);

%% ========================================================================
%  TEST 5: Closed-Loop PD Attitude Step (10 deg Roll & Pitch at t = 1.0 s)
%  ========================================================================
fprintf('Running Test 5: Closed-Loop PD Attitude Step Response...\n');
m = 2.00; assignin('base', 'm', m); assignin('base', 'test_case', 5);
out5 = sim(modelName, 'StopTime', '5.0');

[t5, Xe5] = extract_ts(out5.X_e_out);
[~,  Eu5] = extract_ts(out5.Euler_out);
[~,  W5]  = extract_ts(out5.omega_rotors_out);

fig5 = figure('Color','w','Position',[100 100 900 750]);
subplot(3,1,1);
plot(t5, rad2deg(Eu5(:,1)), 'b', 'LineWidth', 1.6); hold on;
plot(t5, rad2deg(Eu5(:,2)), 'r--', 'LineWidth', 1.6);
plot(t5, rad2deg(Eu5(:,3)), 'k:', 'LineWidth', 1.3);
yline(10, 'g--', 'LineWidth', 1.1); grid on;
ylabel('Euler Angles (deg)'); title('Test 5: Closed-Loop PD Attitude Step Response (10^\circ Roll & Pitch)');
legend('\phi (Roll)', '\theta (Pitch)', '\psi (Yaw)', '10^\circ Reference', 'Location', 'southeast');

subplot(3,1,2);
plot(t5, -Xe5(:,3), 'b', 'LineWidth', 1.6); hold on;
yline(10, 'g--', 'LineWidth', 1.1); grid on;
ylabel('Altitude -Z_e (m)'); ylim([9.8 10.2]);
legend('Actual Altitude', '10 m Reference', 'Location', 'best');

subplot(3,1,3);
plot(t5, W5, 'LineWidth', 1.5); hold on;
yline(omega_max, 'k--', 'LineWidth', 1.2); grid on;
xlabel('Time (s)'); ylabel('Rotor Speed (rad/s)');
legend('\omega_1', '\omega_2', '\omega_3', '\omega_4', '\omega_{max}', 'Location', 'east');
ylim([520 970]);

exportgraphics(fig5, fullfile(dirs{5}, 'test5_pd_control_response.png'), 'Resolution', 300);
fprintf('All 5 Step-5 tests completed and saved to %s\n', baseDir);

%% ========================================================================
%  Helper: Safely convert 2D or 3D Simulink Timeseries into [N x M] matrix
%  ========================================================================
function [t, Y] = extract_ts(ts)
    t = ts.Time(:);
    raw = squeeze(ts.Data);
    if size(raw, 1) ~= numel(t) && size(raw, 2) == numel(t)
        Y = raw.';
    else
        Y = raw;
    end
end