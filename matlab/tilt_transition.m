%% REI Drone - VTOL-to-Cruise Tilt Transition Analysis
%  Rootcastle Engineering & Innovation
%
%  Simulates the critical transition phase where the REI Drone tilt
%  mechanisms rotate the motors from vertical (hover) to horizontal
%  (cruise) orientation. Models the simultaneous changes in thrust
%  vector, aerodynamic lift buildup, altitude maintenance, and airspeed.
%
%  Author: Batuhan Ayribas, M.Sc.

figDir = getappdata(0, 'figDir');
if isempty(figDir)
    figDir = fullfile(fileparts(mfilename('fullpath')), '..', 'figures');
    if ~exist(figDir, 'dir'), mkdir(figDir); end
end

%% Parameters
m = 2.0;                          % Vehicle mass (kg)
g = 9.81;                         % Gravity (m/s^2)
W = m * g;                        % Weight (N)
rho = 1.225;                      % Air density
S = 0.12;                         % Wing area (m^2)
CL_alpha = 4.5;                   % Lift curve slope (1/rad)
CD_0 = 0.035;                     % Parasitic drag
AR = 8.33;                        % Aspect ratio
e_osw = 0.78;                     % Oswald efficiency
T_max = 32;                       % Maximum total thrust (N)

%% Simulation Setup
dt = 0.01;                        % Time step (s)
t_end = 15;                       % Total simulation time (s)
t = 0:dt:t_end;
N = length(t);

% State variables
alt = zeros(1, N); alt(1) = 20;   % Start at 20 m altitude
Vx = zeros(1, N);                 % Forward airspeed (m/s)
Vz = zeros(1, N);                 % Vertical speed (m/s, positive = up)
tilt = zeros(1, N);               % Tilt angle (deg, 0=vertical, 90=horizontal)
aoa = zeros(1, N);                % Angle of attack (deg)
L_aero = zeros(1, N);             % Aerodynamic lift (N)
D_aero = zeros(1, N);             % Aerodynamic drag (N)
T_cmd = zeros(1, N);              % Commanded thrust (N)

%% Transition Profile
% Phase 1: Hover stabilize (0-3s)
% Phase 2: Begin tilt, accelerate (3-8s)
% Phase 3: Full cruise (8-15s)

tilt_start = 3.0;                  % Start tilt at t=3s
tilt_end = 8.0;                    % Complete tilt at t=8s
tilt_rate = 90 / (tilt_end - tilt_start);  % degrees per second

for k = 1:N-1
    % Tilt angle profile (smooth S-curve)
    if t(k) < tilt_start
        tilt(k) = 0;
    elseif t(k) < tilt_end
        progress = (t(k) - tilt_start) / (tilt_end - tilt_start);
        % Smooth S-curve: 3*p^2 - 2*p^3
        s_curve = 3 * progress^2 - 2 * progress^3;
        tilt(k) = 90 * s_curve;
    else
        tilt(k) = 90;
    end

    tilt_rad = deg2rad(tilt(k));

    % Thrust vector decomposition
    T_vertical = T_cmd(k) * cos(tilt_rad);
    T_horizontal = T_cmd(k) * sin(tilt_rad);

    % Aerodynamic forces
    V = sqrt(Vx(k)^2 + Vz(k)^2);
    if V > 0.5
        aoa_rad = atan2(Vz(k), Vx(k));
        aoa(k) = rad2deg(aoa_rad);
        CL = CL_alpha * aoa_rad;
        CL = max(min(CL, 1.35), -0.5);
        CD = CD_0 + CL^2 / (pi * e_osw * AR);
        q_dyn = 0.5 * rho * Vx(k)^2;
        L_aero(k) = q_dyn * S * CL;
        D_aero(k) = q_dyn * S * CD;
    else
        L_aero(k) = 0;
        D_aero(k) = 0;
    end

    % Altitude control: adjust thrust to maintain altitude
    alt_error = 20 - alt(k);
    vz_error = 0 - Vz(k);
    T_needed_vert = W - L_aero(k) + 3.0 * alt_error + 2.0 * vz_error;

    if cos(tilt_rad) > 0.1
        T_cmd_new = T_needed_vert / cos(tilt_rad);
    else
        T_cmd_new = T_max * 0.7;  % During high tilt, use nominal thrust
    end
    T_cmd(k) = max(min(T_cmd_new, T_max), 0);

    % Recalculate after thrust adjustment
    T_vertical = T_cmd(k) * cos(tilt_rad);
    T_horizontal = T_cmd(k) * sin(tilt_rad);

    % Equations of motion
    ax = (T_horizontal - D_aero(k)) / m;
    az = (T_vertical + L_aero(k) - W) / m;

    Vx(k+1) = Vx(k) + ax * dt;
    Vz(k+1) = Vz(k) + az * dt;
    alt(k+1) = alt(k) + Vz(k) * dt;

    if k < N
        tilt(k+1) = tilt(k);
        T_cmd(k+1) = T_cmd(k);
    end
end

V_airspeed = Vx * 3.6;  % Convert to km/h

fprintf('  Tilt Transition Results:\n');
fprintf('    Transition duration: %.1f s\n', tilt_end - tilt_start);
fprintf('    Final cruise speed: %.1f km/h\n', V_airspeed(end));
fprintf('    Max altitude deviation: %.2f m\n', max(abs(alt - 20)));
fprintf('    Final altitude: %.1f m\n', alt(end));

%% Figure 1: Transition Overview
fig1 = figure('Position', [100 100 1200 700], 'Color', 'w');

subplot(2,3,1);
plot(t, tilt(1:N), 'b-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Tilt Angle (deg)');
title('Motor Tilt Angle'); grid on; ylim([-5 95]);

subplot(2,3,2);
plot(t, V_airspeed, 'b-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Airspeed (km/h)');
title('Forward Airspeed'); grid on;

subplot(2,3,3);
plot(t, alt, 'b-', 'LineWidth', 2); hold on;
yline(20, 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Altitude (m)');
title('Altitude Hold During Transition'); grid on;
legend('Actual', 'Target');

subplot(2,3,4);
plot(t, T_cmd, 'b-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Thrust (N)');
title('Total Commanded Thrust'); grid on;

subplot(2,3,5);
plot(t, L_aero, 'b-', 'LineWidth', 2); hold on;
plot(t, D_aero, 'r-', 'LineWidth', 2);
yline(W, 'k--', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('Force (N)');
title('Aerodynamic Forces'); grid on;
legend('Lift', 'Drag', 'Weight');

subplot(2,3,6);
plot(t, Vz, 'b-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Vertical Speed (m/s)');
title('Climb / Sink Rate'); grid on;

sgtitle('REI Drone - VTOL to Cruise Transition', 'FontSize', 14, 'FontWeight', 'bold');
saveas(fig1, fullfile(figDir, 'tilt_transition.png'));

%% Figure 2: Thrust Vector Decomposition
fig2 = figure('Position', [100 100 800 400], 'Color', 'w');

T_vert_comp = T_cmd .* cosd(tilt(1:N));
T_horz_comp = T_cmd .* sind(tilt(1:N));

plot(t, T_vert_comp, 'b-', 'LineWidth', 2); hold on;
plot(t, T_horz_comp, 'r-', 'LineWidth', 2);
plot(t, T_cmd, 'k--', 'LineWidth', 1.5);
yline(W, 'g--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Force (N)');
title('REI Drone - Thrust Vector Decomposition During Transition', ...
    'FontSize', 14, 'FontWeight', 'bold');
legend('Vertical Component', 'Horizontal Component', 'Total Thrust', 'Vehicle Weight');
grid on;
saveas(fig2, fullfile(figDir, 'thrust_vector_decomposition.png'));
