%% REI Drone - Motor and Propulsion Analysis
%  Rootcastle Engineering & Innovation
%
%  Analyzes the 2216 880KV brushless motors paired with 10x4.5 three-blade
%  propellers. Covers thrust curves, power consumption, motor efficiency,
%  and propulsive efficiency across the throttle range.
%
%  Author: Batuhan Ayribas, M.Sc.

figDir = getappdata(0, 'figDir');
if isempty(figDir)
    figDir = fullfile(fileparts(mfilename('fullpath')), '..', 'figures');
    if ~exist(figDir, 'dir'), mkdir(figDir); end
end

if ~evalin('base', 'exist(''drone_params'', ''var'')')
    run('weight_cg_analysis');
end
dp = evalin('base', 'drone_params');

%% Motor Parameters
KV = dp.motor_kv;                       % Motor KV rating (RPM/V)
V_batt = dp.battery_voltage_nom;        % Battery nominal voltage (V)
R_motor = 0.085;                        % Motor winding resistance (Ohm)
I_no_load = 0.5;                        % No-load current (A)
D_prop = dp.prop_diameter_m;            % Propeller diameter (m)
num_blades = 3;                         % Number of blades
rho = 1.225;                            % Air density (kg/m^3)
m = dp.total_mass_kg;
g = 9.81;

% Propeller coefficients for 10x4.5 3-blade (empirical fit)
CT_static = 0.012;                      % Static thrust coefficient
CP_static = 0.0045;                     % Static power coefficient

%% Throttle Sweep
throttle = linspace(0, 1, 100);
V_applied = throttle * V_batt;

% Motor RPM (simplified model: RPM = KV * V_applied * efficiency factor)
RPM = KV * V_applied * 0.85;
n = RPM / 60;                           % Revolutions per second

% Thrust per motor (T = CT * rho * n^2 * D^4)
T_per_motor = CT_static * rho * n.^2 * D_prop^4;

% Torque per motor (Q = CQ * rho * n^2 * D^5)
CQ_static = CP_static / (2 * pi);
Q_per_motor = CQ_static * rho * n.^2 * D_prop^5;

% Mechanical power (P_mech = 2*pi*n*Q)
P_mech = 2 * pi * n .* Q_per_motor;

% Electrical power and current
I_motor = I_no_load + Q_per_motor ./ (R_motor * 10);  % Simplified current model
I_motor = max(I_motor, I_no_load);
P_elec = V_applied .* I_motor;
P_elec(P_elec == 0) = 0.001;

% Motor efficiency
eta_motor = P_mech ./ P_elec;
eta_motor(throttle < 0.05) = 0;
eta_motor = min(eta_motor, 0.95);

% Total system values (4 motors)
T_total = 4 * T_per_motor;
P_total = 4 * P_elec;
I_total = 4 * I_motor;

% Hover throttle estimation
W = m * g;
hover_idx = find(T_total >= W, 1, 'first');
if isempty(hover_idx), hover_idx = 50; end
hover_throttle = throttle(hover_idx);
hover_power = P_total(hover_idx);
hover_current = I_total(hover_idx);

% Thrust-to-weight ratio at full throttle
TWR = T_total(end) / W;

fprintf('  Propulsion Analysis Results:\n');
fprintf('    Max thrust per motor: %.2f N (%.0f g)\n', T_per_motor(end), T_per_motor(end)*1000/g);
fprintf('    Max total thrust: %.2f N (TWR = %.2f)\n', T_total(end), TWR);
fprintf('    Hover throttle: %.0f%%\n', hover_throttle*100);
fprintf('    Hover power: %.0f W (%.1f A total)\n', hover_power, hover_current);
fprintf('    Peak motor efficiency: %.1f%%\n', max(eta_motor)*100);

%% Figure 1: Thrust and Power Curves
fig1 = figure('Position', [100 100 1200 500], 'Color', 'w');

subplot(1,3,1);
plot(throttle*100, T_per_motor, 'b-', 'LineWidth', 2); hold on;
plot(throttle*100, T_total, 'r-', 'LineWidth', 2);
yline(W, 'k--', 'LineWidth', 1.5);
plot(hover_throttle*100, W, 'gp', 'MarkerSize', 15, 'MarkerFaceColor', 'g');
xlabel('Throttle (%)'); ylabel('Thrust (N)');
title('Thrust vs Throttle'); grid on;
legend('Single Motor', 'Total (4 Motors)', 'Vehicle Weight', 'Hover Point', 'Location', 'northwest');

subplot(1,3,2);
plot(throttle*100, P_elec, 'b-', 'LineWidth', 2); hold on;
plot(throttle*100, P_total, 'r-', 'LineWidth', 2);
plot(hover_throttle*100, hover_power, 'gp', 'MarkerSize', 15, 'MarkerFaceColor', 'g');
xlabel('Throttle (%)'); ylabel('Power (W)');
title('Electrical Power'); grid on;
legend('Single Motor', 'Total System', 'Hover Point', 'Location', 'northwest');

subplot(1,3,3);
plot(throttle*100, eta_motor*100, 'b-', 'LineWidth', 2);
xlabel('Throttle (%)'); ylabel('Efficiency (%)');
title('Motor Efficiency'); grid on; ylim([0 100]);

sgtitle('REI Drone - Motor & Propulsion Performance (2216 880KV + 10x4.5")', ...
    'FontSize', 14, 'FontWeight', 'bold');
saveas(fig1, fullfile(figDir, 'motor_propulsion.png'));

%% Figure 2: RPM and Current
fig2 = figure('Position', [100 100 1000 400], 'Color', 'w');

subplot(1,2,1);
plot(throttle*100, RPM, 'b-', 'LineWidth', 2);
xlabel('Throttle (%)'); ylabel('RPM');
title('Motor Speed'); grid on;

subplot(1,2,2);
plot(throttle*100, I_motor, 'b-', 'LineWidth', 2); hold on;
plot(throttle*100, I_total, 'r-', 'LineWidth', 2);
xlabel('Throttle (%)'); ylabel('Current (A)');
title('Current Draw'); grid on;
legend('Single Motor', 'Total (4 Motors)', 'Location', 'northwest');

sgtitle('REI Drone - Motor Operating Characteristics', 'FontSize', 14, 'FontWeight', 'bold');
saveas(fig2, fullfile(figDir, 'motor_rpm_current.png'));
