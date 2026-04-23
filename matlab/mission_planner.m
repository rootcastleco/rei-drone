%% REI Drone - Autonomous Mission Planner Simulation
%  Rootcastle Engineering & Innovation
%
%  Simulates an autonomous survey mission with GPS waypoint navigation.
%  The mission includes takeoff, waypoint following, and return-to-launch.
%  Generates a coverage map and flight path visualization.
%
%  Author: Batuhan Ayribas, M.Sc.

figDir = getappdata(0, 'figDir');
if isempty(figDir)
    figDir = fullfile(fileparts(mfilename('fullpath')), '..', 'figures');
    if ~exist(figDir, 'dir'), mkdir(figDir); end
end

%% Mission Parameters
cruise_speed = 14;                  % Cruise speed (m/s) ~ 50 km/h
hover_alt = 50;                     % Survey altitude (m)
turn_rate = 15;                     % Max turn rate (deg/s)
dt = 0.5;                          % Simulation time step (s)

%% Define Waypoints (local NED coordinates relative to launch point)
% Survey pattern: rectangular grid with 50m spacing
survey_width = 400;                 % Survey area width (m)
survey_length = 600;                % Survey area length (m)
line_spacing = 50;                  % Spacing between survey lines (m)

% Generate lawnmower pattern waypoints
waypoints = [0, 0, hover_alt];      % Launch point

% Add takeoff waypoint
waypoints = [waypoints; 0, 0, hover_alt];

% Generate survey lines
n_lines = floor(survey_width / line_spacing) + 1;
for i = 1:n_lines
    y_pos = (i-1) * line_spacing;
    if mod(i, 2) == 1
        % Forward pass
        waypoints = [waypoints; 0, y_pos, hover_alt];
        waypoints = [waypoints; survey_length, y_pos, hover_alt];
    else
        % Reverse pass
        waypoints = [waypoints; survey_length, y_pos, hover_alt];
        waypoints = [waypoints; 0, y_pos, hover_alt];
    end
end

% Return to launch
waypoints = [waypoints; 0, 0, hover_alt];
waypoints = [waypoints; 0, 0, 0];

n_wp = size(waypoints, 1);

%% Simulate Flight Path
% Simple waypoint following with constant speed
pos = [0, 0, 0];                    % Starting position
heading = 0;                        % Initial heading (rad)
path_x = pos(1); path_y = pos(2); path_z = pos(3);
path_t = 0;
current_wp = 2;                     % Target waypoint index

max_time = 3600;                    % Max simulation time (s)
t_sim = 0;
wp_times = zeros(n_wp, 1);         % Time at each waypoint

while current_wp <= n_wp && t_sim < max_time
    target = waypoints(current_wp, :);
    dx = target(1) - pos(1);
    dy = target(2) - pos(2);
    dz = target(3) - pos(3);
    dist = sqrt(dx^2 + dy^2 + dz^2);

    if dist < 3.0  % Waypoint reached (3m acceptance radius)
        wp_times(current_wp) = t_sim;
        current_wp = current_wp + 1;
        continue;
    end

    % Desired heading
    heading_desired = atan2(dy, dx);

    % Adjust heading with turn rate limit
    heading_error = heading_desired - heading;
    heading_error = atan2(sin(heading_error), cos(heading_error));
    max_turn = deg2rad(turn_rate) * dt;
    heading = heading + max(min(heading_error, max_turn), -max_turn);

    % Velocity components
    if dist > 10
        speed = cruise_speed;
    else
        speed = max(cruise_speed * dist / 10, 2);
    end

    vx = speed * cos(heading);
    vy = speed * sin(heading);

    % Altitude control
    alt_error = target(3) - pos(3);
    vz = max(min(alt_error * 0.5, 3), -3);

    % Update position
    pos(1) = pos(1) + vx * dt;
    pos(2) = pos(2) + vy * dt;
    pos(3) = pos(3) + vz * dt;
    t_sim = t_sim + dt;

    path_x = [path_x, pos(1)];
    path_y = [path_y, pos(2)];
    path_z = [path_z, pos(3)];
    path_t = [path_t, t_sim];
end

total_distance = sum(sqrt(diff(path_x).^2 + diff(path_y).^2 + diff(path_z).^2));
mission_time = t_sim;

fprintf('  Mission Planner Results:\n');
fprintf('    Waypoints: %d\n', n_wp);
fprintf('    Total distance: %.0f m (%.1f km)\n', total_distance, total_distance/1000);
fprintf('    Mission time: %.0f s (%.1f min)\n', mission_time, mission_time/60);
fprintf('    Average speed: %.1f m/s (%.1f km/h)\n', ...
    total_distance/mission_time, total_distance/mission_time*3.6);

%% Figure 1: Mission Path (Top View)
fig1 = figure('Position', [100 100 1000 700], 'Color', 'w');

subplot(2,2,[1,3]);
plot(path_x, path_y, 'b-', 'LineWidth', 1.5); hold on;
plot(waypoints(:,1), waypoints(:,2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
plot(path_x(1), path_y(1), 'gs', 'MarkerSize', 15, 'MarkerFaceColor', 'g');
plot(path_x(end), path_y(end), 'rs', 'MarkerSize', 15, 'MarkerFaceColor', 'r');

% Label waypoints
for i = 1:n_wp
    text(waypoints(i,1)+5, waypoints(i,2)+5, sprintf('WP%d', i), 'FontSize', 7);
end

% Draw survey area boundary
rectangle('Position', [-20, -20, survey_length+40, survey_width+40], ...
    'EdgeColor', [0.5 0.5 0.5], 'LineStyle', '--', 'LineWidth', 1);

xlabel('North (m)'); ylabel('East (m)');
title('Survey Mission Path (Top View)'); grid on; axis equal;
legend('Flight Path', 'Waypoints', 'Launch', 'Landing', 'Location', 'best');

subplot(2,2,2);
plot(path_t/60, path_z, 'b-', 'LineWidth', 2);
xlabel('Time (min)'); ylabel('Altitude (m)');
title('Altitude Profile'); grid on;

subplot(2,2,4);
speed_profile = sqrt(diff(path_x).^2 + diff(path_y).^2) / dt * 3.6;
plot(path_t(2:end)/60, speed_profile, 'b-', 'LineWidth', 1);
xlabel('Time (min)'); ylabel('Ground Speed (km/h)');
title('Speed Profile'); grid on;

sgtitle(sprintf('REI Drone - Autonomous Survey Mission (%.0fx%.0f m, %.1f min)', ...
    survey_length, survey_width, mission_time/60), 'FontSize', 14, 'FontWeight', 'bold');
saveas(fig1, fullfile(figDir, 'mission_planner.png'));

%% Figure 2: 3D Flight Path
fig2 = figure('Position', [100 100 700 500], 'Color', 'w');
plot3(path_x, path_y, path_z, 'b-', 'LineWidth', 1.5); hold on;
plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'ro', ...
    'MarkerSize', 8, 'MarkerFaceColor', 'r');
plot3(path_x(1), path_y(1), path_z(1), 'gs', 'MarkerSize', 15, 'MarkerFaceColor', 'g');
xlabel('North (m)'); ylabel('East (m)'); zlabel('Altitude (m)');
title('REI Drone - 3D Mission Trajectory', 'FontSize', 14, 'FontWeight', 'bold');
grid on; view(45, 30);
legend('Flight Path', 'Waypoints', 'Launch Point');
saveas(fig2, fullfile(figDir, 'mission_3d_trajectory.png'));
