%% REI Drone - 6-DOF Flight Dynamics Simulation
%  Rootcastle Engineering & Innovation
%
%  Simulates the full six degree-of-freedom dynamics of the REI Drone
%  in hover mode. Includes attitude response to step commands and
%  disturbance rejection. Uses the rigid body equations of motion
%  with a simplified PID attitude controller.
%
%  Author: Batuhan Ayribas, M.Sc.

figDir = getappdata(0, 'figDir');
if isempty(figDir)
    figDir = fullfile(fileparts(mfilename('fullpath')), '..', 'figures');
    if ~exist(figDir, 'dir'), mkdir(figDir); end
end

%% Vehicle Parameters
m = 2.0;                           % Vehicle mass (kg)
g_acc = 9.81;                      % Gravity (m/s^2)
L_arm = 0.25;                      % Motor arm length from CG (m)

% Moments of inertia (estimated for X-frame quad)
Ixx = 0.015;                       % Roll inertia (kg.m^2)
Iyy = 0.015;                       % Pitch inertia (kg.m^2)
Izz = 0.025;                       % Yaw inertia (kg.m^2)

%% Simulation Parameters
dt = 0.001;                        % Time step (s)
t_end = 10;                        % Simulation duration (s)
t = 0:dt:t_end;
N = length(t);

%% State Vector: [x, y, z, u, v, w, phi, theta, psi, p, q, r]
state = zeros(12, N);
state(3, 1) = -10;                 % Start at 10m altitude (NED, z is down)

%% Control Inputs (step commands)
% Apply a 10-degree roll step at t=2s, return to level at t=5s
% Apply a 5-degree pitch step at t=3s, return at t=6s
phi_cmd = zeros(1, N);
theta_cmd = zeros(1, N);
psi_cmd = zeros(1, N);
z_cmd = -10 * ones(1, N);          % Hold altitude at 10m

phi_cmd(t >= 2 & t < 5) = deg2rad(10);
theta_cmd(t >= 3 & t < 6) = deg2rad(5);

%% PID Gains (attitude rate control)
% Roll rate
Kp_p = 6.0; Ki_p = 1.0; Kd_p = 0.15;
% Pitch rate
Kp_q = 6.0; Ki_q = 1.0; Kd_q = 0.15;
% Yaw rate
Kp_r = 4.0; Ki_r = 0.5; Kd_r = 0.1;
% Altitude
Kp_z = 3.0; Ki_z = 0.5; Kd_z = 1.5;
% Attitude
Kp_att = 5.0;

%% Error Integrals
int_ep = 0; int_eq = 0; int_er = 0; int_ez = 0;
prev_ep = 0; prev_eq = 0; prev_er = 0; prev_ez = 0;

%% Control Outputs Storage
ctrl_out = zeros(4, N);            % [T_total, tau_roll, tau_pitch, tau_yaw]

%% Simulation Loop
for k = 1:N-1
    % Extract states
    x = state(1,k); y = state(2,k); z = state(3,k);
    u = state(4,k); v = state(5,k); w = state(6,k);
    phi = state(7,k); theta = state(8,k); psi = state(9,k);
    p = state(10,k); q = state(11,k); r = state(12,k);

    % Outer loop: attitude angle to rate command
    p_cmd = Kp_att * (phi_cmd(k) - phi);
    q_cmd = Kp_att * (theta_cmd(k) - theta);
    r_cmd = Kp_att * (psi_cmd(k) - psi);

    % Inner loop: rate PID
    ep = p_cmd - p; int_ep = int_ep + ep * dt;
    dep = (ep - prev_ep) / dt; prev_ep = ep;
    tau_phi = Kp_p * ep + Ki_p * int_ep + Kd_p * dep;

    eq = q_cmd - q; int_eq = int_eq + eq * dt;
    deq = (eq - prev_eq) / dt; prev_eq = eq;
    tau_theta = Kp_q * eq + Ki_q * int_eq + Kd_q * deq;

    er = r_cmd - r; int_er = int_er + er * dt;
    der = (er - prev_er) / dt; prev_er = er;
    tau_psi = Kp_r * er + Ki_r * int_er + Kd_r * der;

    % Altitude PID
    ez = z_cmd(k) - z; int_ez = int_ez + ez * dt;
    dez = (ez - prev_ez) / dt; prev_ez = ez;
    T = m * g_acc + Kp_z * ez + Ki_z * int_ez + Kd_z * dez;
    T = max(T, 0);

    ctrl_out(:, k) = [T; tau_phi; tau_theta; tau_psi];

    % Add wind disturbance at t=7s
    F_dist = [0; 0; 0];
    if t(k) >= 7 && t(k) < 7.5
        F_dist = [2.0; 1.0; 0.5];  % Wind gust (N)
    end

    % Rotation matrix (simplified for small angles)
    cphi = cos(phi); sphi = sin(phi);
    cth = cos(theta); sth = sin(theta);
    cpsi = cos(psi); spsi = sin(psi);

    % Forces in body frame
    Fx = -m * g_acc * sth + F_dist(1);
    Fy = m * g_acc * cth * sphi + F_dist(2);
    Fz = m * g_acc * cth * cphi - T + F_dist(3);

    % Translational acceleration
    du = Fx / m + r * v - q * w;
    dv = Fy / m - r * u + p * w;
    dw = Fz / m + q * u - p * v;

    % Rotational acceleration (Euler's equations)
    dp = (tau_phi + (Iyy - Izz) * q * r) / Ixx;
    dq = (tau_theta + (Izz - Ixx) * p * r) / Iyy;
    dr = (tau_psi + (Ixx - Iyy) * p * q) / Izz;

    % Euler angle rates
    dphi = p + (q * sphi + r * cphi) * sth / cth;
    dtheta = q * cphi - r * sphi;
    dpsi = (q * sphi + r * cphi) / cth;

    % Position rates (body to NED)
    dx = cth*cpsi*u + (sphi*sth*cpsi - cphi*spsi)*v + (cphi*sth*cpsi + sphi*spsi)*w;
    dy = cth*spsi*u + (sphi*sth*spsi + cphi*cpsi)*v + (cphi*sth*spsi - sphi*cpsi)*w;
    dz = -sth*u + sphi*cth*v + cphi*cth*w;

    % Integrate
    state(:, k+1) = state(:, k) + dt * [dx; dy; dz; du; dv; dw; dphi; dtheta; dpsi; dp; dq; dr];
end

fprintf('  Flight Dynamics: Simulation completed (%.0f ms, %d steps)\n', t_end*1000, N);

%% Figure 1: Attitude Response
fig1 = figure('Position', [100 100 1200 600], 'Color', 'w');

subplot(2,3,1);
plot(t, rad2deg(state(7,:)), 'b-', 'LineWidth', 1.5); hold on;
plot(t, rad2deg(phi_cmd), 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Roll (deg)');
title('Roll Response'); grid on; legend('Actual', 'Command');

subplot(2,3,2);
plot(t, rad2deg(state(8,:)), 'b-', 'LineWidth', 1.5); hold on;
plot(t, rad2deg(theta_cmd), 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Pitch (deg)');
title('Pitch Response'); grid on; legend('Actual', 'Command');

subplot(2,3,3);
plot(t, rad2deg(state(9,:)), 'b-', 'LineWidth', 1.5); hold on;
plot(t, rad2deg(psi_cmd), 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Yaw (deg)');
title('Yaw Response'); grid on; legend('Actual', 'Command');

subplot(2,3,4);
plot(t, rad2deg(state(10,:)), 'b-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Roll Rate (deg/s)');
title('Roll Rate'); grid on;

subplot(2,3,5);
plot(t, rad2deg(state(11,:)), 'b-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Pitch Rate (deg/s)');
title('Pitch Rate'); grid on;

subplot(2,3,6);
plot(t, rad2deg(state(12,:)), 'b-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Yaw Rate (deg/s)');
title('Yaw Rate'); grid on;

sgtitle('REI Drone - 6-DOF Attitude Dynamics', 'FontSize', 14, 'FontWeight', 'bold');
saveas(fig1, fullfile(figDir, 'flight_dynamics_attitude.png'));

%% Figure 2: Position Response
fig2 = figure('Position', [100 100 1200 400], 'Color', 'w');

subplot(1,3,1);
plot(t, state(1,:), 'b-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('X North (m)');
title('North Position'); grid on;

subplot(1,3,2);
plot(t, state(2,:), 'b-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Y East (m)');
title('East Position'); grid on;

subplot(1,3,3);
plot(t, -state(3,:), 'b-', 'LineWidth', 1.5); hold on;
plot(t, -z_cmd, 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Altitude (m)');
title('Altitude Hold'); grid on; legend('Actual', 'Command');

sgtitle('REI Drone - Position Response with Wind Disturbance at t=7s', ...
    'FontSize', 14, 'FontWeight', 'bold');
saveas(fig2, fullfile(figDir, 'flight_dynamics_position.png'));
