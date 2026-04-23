%% REI Drone - Aerodynamic Performance Analysis
%  Rootcastle Engineering & Innovation
%
%  Characterizes the aerodynamic performance of the REI Drone wing and
%  fuselage. The wing has a 1000 mm span integrated into the motor arms.
%  Analysis covers lift, drag polar, L/D ratio, stall, and cruise envelope.
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

%% Parameters
rho = 1.225; mu = 1.789e-5;
S = dp.wing_area_m2; b = dp.wingspan_m; c = dp.wing_chord_m;
AR = dp.AR; e = 0.78;
m = dp.total_mass_kg; g = 9.81; W = m * g;

CL_alpha = 2 * pi * (AR / (AR + 2));
alpha_0L = -2; CL_max = 1.35; CD_0 = 0.035;

%% Angle of Attack Sweep
alpha = linspace(-5, 20, 200);
CL = CL_alpha * deg2rad(alpha - alpha_0L);
alpha_stall = alpha_0L + rad2deg(CL_max / CL_alpha);
for i = 1:length(alpha)
    if alpha(i) > alpha_stall
        da = alpha(i) - alpha_stall;
        CL(i) = CL_max * exp(-0.08 * da^2);
    end
end
CD = CD_0 + CL.^2 / (pi * e * AR);
LD = CL ./ CD; LD(CD < 0.001) = 0;

%% Speed Analysis
V = linspace(5, 30, 200);
Re = rho * V * c / mu;
CL_req = (2*W) ./ (rho * V.^2 * S);
CD_lev = CD_0 + CL_req.^2 / (pi * e * AR);
D_lev = 0.5 * rho * V.^2 .* S .* CD_lev;
P_req = D_lev .* V;
V_stall = sqrt(2*W / (rho*S*CL_max));
[P_min, iP] = min(P_req); V_end = V(iP);
[~, iLD] = max(LD);
[D_min, iD] = min(D_lev); V_best = V(iD);

fprintf('  Aerodynamics: Vstall=%.1f m/s, L/Dmax=%.1f, Pmin=%.0f W\n', V_stall, max(LD), P_min);

%% Figures
fig1 = figure('Position', [100 100 1200 500], 'Color', 'w');
subplot(1,3,1);
plot(alpha, CL, 'b-', 'LineWidth', 2); hold on;
xline(alpha_stall, 'r--', 'LineWidth', 1.5);
xlabel('Angle of Attack (deg)'); ylabel('C_L');
title('Lift Curve'); grid on;

subplot(1,3,2);
plot(CD, CL, 'b-', 'LineWidth', 2);
xlabel('C_D'); ylabel('C_L');
title('Drag Polar'); grid on;

subplot(1,3,3);
plot(alpha, LD, 'b-', 'LineWidth', 2); hold on;
plot(alpha(iLD), max(LD), 'rp', 'MarkerSize', 15, 'MarkerFaceColor', 'r');
xlabel('Angle of Attack (deg)'); ylabel('L/D');
title('Aerodynamic Efficiency'); grid on;

sgtitle('REI Drone - Aerodynamic Characteristics', 'FontSize', 15, 'FontWeight', 'bold');
saveas(fig1, fullfile(figDir, 'aerodynamics_coefficients.png'));

fig2 = figure('Position', [100 100 1100 500], 'Color', 'w');
subplot(1,2,1);
plot(V*3.6, P_req, 'b-', 'LineWidth', 2); hold on;
plot(V_end*3.6, P_min, 'rp', 'MarkerSize', 15, 'MarkerFaceColor', 'r');
xline(V_stall*3.6, 'r--');
xlabel('Airspeed (km/h)'); ylabel('Power Required (W)');
title('Power vs Airspeed'); grid on;

subplot(1,2,2);
plot(V*3.6, D_lev, 'b-', 'LineWidth', 2); hold on;
plot(V_best*3.6, D_min, 'rp', 'MarkerSize', 15, 'MarkerFaceColor', 'r');
xlabel('Airspeed (km/h)'); ylabel('Drag Force (N)');
title('Drag vs Airspeed'); grid on;

sgtitle('REI Drone - Flight Performance', 'FontSize', 15, 'FontWeight', 'bold');
saveas(fig2, fullfile(figDir, 'aerodynamics_performance.png'));
