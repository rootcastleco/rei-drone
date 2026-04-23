%% REI Drone - Structural Load Analysis
%  Rootcastle Engineering & Innovation
%
%  Analyzes the structural loads on the REI Drone airframe, focusing on
%  motor arm bending stress, wing spar loading, and vibration modes.
%  Uses beam theory and simplified FEM-like discretization.
%
%  Author: Batuhan Ayribas, M.Sc.

figDir = getappdata(0, 'figDir');
if isempty(figDir)
    figDir = fullfile(fileparts(mfilename('fullpath')), '..', 'figures');
    if ~exist(figDir, 'dir'), mkdir(figDir); end
end

%% Material Properties (Carbon Fiber Composite)
E = 70e9;                          % Young's modulus (Pa)
sigma_yield = 600e6;               % Yield strength (Pa)
rho_mat = 1600;                    % Material density (kg/m^3)

%% Motor Arm Analysis
% Each arm is a cantilever beam, fixed at the fuselage, motor at the tip
L_arm = 0.25;                      % Arm length (m)
% Arm cross-section: hollow circular tube
D_outer = 0.016;                   % Outer diameter (m)
D_inner = 0.012;                   % Inner diameter (m)
I_arm = pi/64 * (D_outer^4 - D_inner^4);   % Second moment of area
A_arm = pi/4 * (D_outer^2 - D_inner^2);    % Cross-sectional area

% Loading: motor thrust at tip + motor weight
T_motor_max = 8.2;                 % Max thrust per motor (N)
W_motor = 0.062 * 9.81;            % Motor weight (N)
W_prop = 0.018 * 9.81;             % Propeller weight (N)
W_tilt = 0.035 * 9.81;             % Tilt mechanism weight (N)
F_tip = T_motor_max + W_motor + W_prop + W_tilt;

% Bending moment distribution along the arm
x_arm = linspace(0, L_arm, 100);
M_arm = F_tip * (L_arm - x_arm);   % Maximum at root

% Bending stress
sigma_arm = M_arm * (D_outer/2) / I_arm;

% Deflection (cantilever with point load)
delta_arm = F_tip * x_arm.^2 .* (3*L_arm - x_arm) / (6*E*I_arm);

% Safety factor
SF_arm = sigma_yield / max(sigma_arm);

% Load factor analysis (1g to 5g)
n_load = linspace(1, 5, 50);
sigma_max_vs_n = max(sigma_arm) * n_load;
SF_vs_n = sigma_yield ./ sigma_max_vs_n;

fprintf('  Structural Analysis Results:\n');
fprintf('    Arm max bending stress: %.1f MPa\n', max(sigma_arm)/1e6);
fprintf('    Arm tip deflection: %.2f mm\n', max(delta_arm)*1000);
fprintf('    Safety factor at 1g: %.1f\n', SF_arm);
fprintf('    Safety factor at 3g: %.1f\n', sigma_yield / (max(sigma_arm)*3));

%% Vibration Analysis
% Natural frequencies of the arm (cantilever beam)
m_arm_per_length = rho_mat * A_arm;  % kg/m
m_tip = (0.062 + 0.018 + 0.035);    % Tip mass (kg)

% First three natural frequencies (cantilever with tip mass)
beta_n = [1.875, 4.694, 7.855];     % Mode shape constants
f_nat = zeros(1, 3);
for n = 1:3
    f_nat(n) = beta_n(n)^2 / (2*pi*L_arm^2) * sqrt(E*I_arm / m_arm_per_length);
end

% Motor excitation frequency at hover RPM
RPM_hover = 6000;
f_motor = RPM_hover / 60;           % Hz
f_blade = f_motor * 3;              % Blade passing frequency (3-blade)

fprintf('    Natural frequencies: f1=%.0f Hz, f2=%.0f Hz, f3=%.0f Hz\n', ...
    f_nat(1), f_nat(2), f_nat(3));
fprintf('    Motor frequency: %.0f Hz, Blade passing: %.0f Hz\n', f_motor, f_blade);

%% Figure 1: Arm Stress and Deflection
fig1 = figure('Position', [100 100 1200 500], 'Color', 'w');

subplot(1,3,1);
plot(x_arm*1000, sigma_arm/1e6, 'b-', 'LineWidth', 2); hold on;
yline(sigma_yield/1e6, 'r--', 'LineWidth', 1.5);
xlabel('Distance from Root (mm)'); ylabel('Bending Stress (MPa)');
title('Arm Bending Stress (Max Thrust)'); grid on;
legend('Stress', 'Yield Strength');

subplot(1,3,2);
plot(x_arm*1000, delta_arm*1000, 'b-', 'LineWidth', 2);
xlabel('Distance from Root (mm)'); ylabel('Deflection (mm)');
title('Arm Deflection'); grid on;

subplot(1,3,3);
plot(n_load, SF_vs_n, 'b-', 'LineWidth', 2); hold on;
yline(1.5, 'r--', 'LineWidth', 1.5);
yline(1.0, 'k--', 'LineWidth', 1.5);
xlabel('Load Factor (g)'); ylabel('Safety Factor');
title('Safety Factor vs Load'); grid on;
legend('Safety Factor', 'Min Recommended (1.5)', 'Failure (1.0)');

sgtitle('REI Drone - Motor Arm Structural Analysis', 'FontSize', 14, 'FontWeight', 'bold');
saveas(fig1, fullfile(figDir, 'structural_arm_analysis.png'));

%% Figure 2: Vibration Analysis
fig2 = figure('Position', [100 100 800 400], 'Color', 'w');

% Campbell diagram: natural frequencies vs RPM
RPM_range = linspace(1000, 12000, 200);
f_motor_range = RPM_range / 60;
f_blade_range = f_motor_range * 3;

plot([RPM_range(1) RPM_range(end)], [f_nat(1) f_nat(1)], 'b-', 'LineWidth', 2); hold on;
plot([RPM_range(1) RPM_range(end)], [f_nat(2) f_nat(2)], 'b--', 'LineWidth', 2);
plot(RPM_range, f_motor_range, 'r-', 'LineWidth', 2);
plot(RPM_range, f_blade_range, 'r--', 'LineWidth', 2);
xline(RPM_hover, 'g-', 'LineWidth', 1.5);

xlabel('Motor RPM'); ylabel('Frequency (Hz)');
title('REI Drone - Campbell Diagram (Vibration Resonance Check)', ...
    'FontSize', 13, 'FontWeight', 'bold');
legend('Arm 1st Mode', 'Arm 2nd Mode', 'Motor Freq (1P)', 'Blade Passing (3P)', ...
    sprintf('Hover RPM (%d)', RPM_hover), 'Location', 'best');
grid on;
saveas(fig2, fullfile(figDir, 'vibration_campbell.png'));
