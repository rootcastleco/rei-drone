%% REI Drone - PID Controller Tuning Analysis
%  Rootcastle Engineering & Innovation
%
%  Evaluates PID controller performance for roll, pitch, and yaw axes.
%  Generates step response characteristics: rise time, settling time,
%  overshoot, and steady-state error. Compares different gain sets.
%
%  Author: Batuhan Ayribas, M.Sc.

figDir = getappdata(0, 'figDir');
if isempty(figDir)
    figDir = fullfile(fileparts(mfilename('fullpath')), '..', 'figures');
    if ~exist(figDir, 'dir'), mkdir(figDir); end
end

%% System Parameters
% Simplified second-order model for each axis: I * ddot_angle = tau
Ixx = 0.015; Iyy = 0.015; Izz = 0.025;

%% Gain Sets to Compare
% Each row: [Kp, Ki, Kd, axis_label, inertia]
gain_sets = {
    % Conservative
    3.0, 0.5, 0.08, 'Conservative', Ixx;
    % Moderate (baseline)
    6.0, 1.0, 0.15, 'Moderate', Ixx;
    % Aggressive
    10.0, 2.0, 0.25, 'Aggressive', Ixx;
};

dt = 0.001;
t_end = 3.0;
t = 0:dt:t_end;
N = length(t);

% Step command: 10 degrees
cmd = deg2rad(10);

%% Simulate Each Gain Set
fig1 = figure('Position', [100 100 1400 500], 'Color', 'w');
colors = {'b', [0 0.6 0], 'r'};
metrics = cell(3, 1);

for gs = 1:3
    Kp = gain_sets{gs, 1};
    Ki = gain_sets{gs, 2};
    Kd = gain_sets{gs, 3};
    label = gain_sets{gs, 4};
    I_axis = gain_sets{gs, 5};

    angle = zeros(1, N);
    rate = zeros(1, N);
    int_err = 0;
    prev_err = 0;
    tau_hist = zeros(1, N);

    for k = 1:N-1
        err = cmd - angle(k);
        int_err = int_err + err * dt;
        d_err = (err - prev_err) / dt;
        prev_err = err;

        tau = Kp * err + Ki * int_err + Kd * d_err;
        tau = max(min(tau, 5.0), -5.0);  % Torque saturation
        tau_hist(k) = tau;

        d_rate = tau / I_axis;
        rate(k+1) = rate(k) + d_rate * dt;
        angle(k+1) = angle(k) + rate(k+1) * dt;
    end

    angle_deg = rad2deg(angle);
    cmd_deg = rad2deg(cmd);

    % Compute metrics
    % Rise time (10% to 90%)
    idx_10 = find(angle_deg >= 0.1 * cmd_deg, 1, 'first');
    idx_90 = find(angle_deg >= 0.9 * cmd_deg, 1, 'first');
    if ~isempty(idx_10) && ~isempty(idx_90)
        rise_time = t(idx_90) - t(idx_10);
    else
        rise_time = NaN;
    end

    % Overshoot
    overshoot = (max(angle_deg) - cmd_deg) / cmd_deg * 100;
    overshoot = max(overshoot, 0);

    % Settling time (within 2% of final value)
    settling_idx = find(abs(angle_deg - cmd_deg) > 0.02 * cmd_deg, 1, 'last');
    if ~isempty(settling_idx)
        settling_time = t(settling_idx);
    else
        settling_time = 0;
    end

    ss_error = abs(angle_deg(end) - cmd_deg);

    metrics{gs} = struct('label', label, 'rise', rise_time, ...
        'overshoot', overshoot, 'settling', settling_time, 'ss_err', ss_error);

    % Plot step response
    subplot(1,3,1);
    plot(t, angle_deg, '-', 'Color', colors{gs}, 'LineWidth', 2); hold on;

    subplot(1,3,2);
    plot(t, rad2deg(rate), '-', 'Color', colors{gs}, 'LineWidth', 2); hold on;

    subplot(1,3,3);
    plot(t, tau_hist, '-', 'Color', colors{gs}, 'LineWidth', 2); hold on;
end

subplot(1,3,1);
yline(rad2deg(cmd), 'k--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Angle (deg)');
title('Roll Step Response'); grid on;
legend('Conservative', 'Moderate', 'Aggressive', 'Command', 'Location', 'best');

subplot(1,3,2);
xlabel('Time (s)'); ylabel('Angular Rate (deg/s)');
title('Roll Rate'); grid on;

subplot(1,3,3);
xlabel('Time (s)'); ylabel('Torque (N.m)');
title('Control Torque'); grid on;

sgtitle('REI Drone - PID Controller Comparison (Roll Axis)', ...
    'FontSize', 14, 'FontWeight', 'bold');
saveas(fig1, fullfile(figDir, 'pid_controller_comparison.png'));

%% Print Metrics Table
fprintf('  PID Tuning Results (Roll Axis, 10 deg step):\n');
fprintf('    %-15s  Rise(s)  OS(%%)   Settle(s)  SS_Err(deg)\n', 'Gain Set');
for gs = 1:3
    m = metrics{gs};
    fprintf('    %-15s  %.3f    %.1f     %.3f      %.3f\n', ...
        m.label, m.rise, m.overshoot, m.settling, m.ss_err);
end

%% Figure 2: Frequency Response (Bode-like analysis)
fig2 = figure('Position', [100 100 1000 500], 'Color', 'w');

freq = logspace(-1, 2, 500);  % 0.1 to 100 Hz
omega = 2 * pi * freq;

for gs = 1:3
    Kp = gain_sets{gs, 1};
    Ki = gain_sets{gs, 2};
    Kd = gain_sets{gs, 3};

    % Open-loop transfer function: G(s) = (Kd*s^2 + Kp*s + Ki) / (I*s^2 * s)
    % Magnitude at each frequency
    s = 1i * omega;
    G = (Kd * s.^2 + Kp * s + Ki) ./ (Ixx * s.^3);
    mag_db = 20 * log10(abs(G));
    phase_deg = rad2deg(angle(G));

    subplot(2,1,1);
    semilogx(freq, mag_db, '-', 'Color', colors{gs}, 'LineWidth', 2); hold on;

    subplot(2,1,2);
    semilogx(freq, phase_deg, '-', 'Color', colors{gs}, 'LineWidth', 2); hold on;
end

subplot(2,1,1);
ylabel('Magnitude (dB)'); title('Open-Loop Bode Plot'); grid on;
legend('Conservative', 'Moderate', 'Aggressive', 'Location', 'best');
ylim([-60 60]);

subplot(2,1,2);
xlabel('Frequency (Hz)'); ylabel('Phase (deg)'); grid on;
yline(-180, 'k--', 'LineWidth', 1);

sgtitle('REI Drone - PID Frequency Response Analysis', ...
    'FontSize', 14, 'FontWeight', 'bold');
saveas(fig2, fullfile(figDir, 'pid_bode_analysis.png'));
