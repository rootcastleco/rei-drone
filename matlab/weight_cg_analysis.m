%% REI Drone - Weight and Center of Gravity Analysis
%  Rootcastle Engineering & Innovation
%
%  This module computes the total vehicle mass, component-level weight
%  breakdown, and the center of gravity location for the REI Drone.
%  Accurate CG estimation is critical for flight stability and control
%  system design. The analysis accounts for all 17 major components
%  identified in the exploded-view assembly diagram.
%
%  Author: Batuhan Ayribas, M.Sc.

figDir = getappdata(0, 'figDir');
if isempty(figDir)
    figDir = fullfile(fileparts(mfilename('fullpath')), '..', 'figures');
    if ~exist(figDir, 'dir'), mkdir(figDir); end
end

%% Component Mass Data
% Each row: [mass_grams, x_position_mm, y_position_mm, z_position_mm]
% Origin is at the nose tip of the fuselage.
% x: forward positive, y: starboard positive, z: downward positive

components = struct();
components.name = {
    'Propeller FL', 'Propeller FR', 'Propeller RL', 'Propeller RR', ...
    'Motor FL', 'Motor FR', 'Motor RL', 'Motor RR', ...
    'Tilt Mechanism FL', 'Tilt Mechanism FR', ...
    'Tilt Mechanism RL', 'Tilt Mechanism RR', ...
    'Fuselage', 'Vertical Stabilizer', ...
    'Wing Left', 'Wing Right', ...
    'Flight Controller', 'Power Distribution Board', ...
    'Telemetry Module', 'Battery', 'Battery Tray', ...
    'Camera / Payload Bay', 'Landing Gear', ...
    'Bottom Cover', 'ESC x4', 'RC Receiver', 'GPS Module', ...
    'Wiring Harness', 'Fasteners and Misc'
};

%        mass(g)   x(mm)   y(mm)   z(mm)
data = [
    18,   150,  -350,   -20;   % Propeller FL
    18,   150,   350,   -20;   % Propeller FR
    18,   450,  -350,   -20;   % Propeller RL
    18,   450,   350,   -20;   % Propeller RR
    62,   150,  -350,     0;   % Motor FL
    62,   150,   350,     0;   % Motor FR
    62,   450,  -350,     0;   % Motor RL
    62,   450,   350,     0;   % Motor RR
    35,   150,  -320,    10;   % Tilt Mechanism FL
    35,   150,   320,    10;   % Tilt Mechanism FR
    35,   450,  -320,    10;   % Tilt Mechanism RL
    35,   450,   320,    10;   % Tilt Mechanism RR
   280,   325,     0,    30;   % Fuselage (Main Body)
    45,   620,     0,   -30;   % Vertical Stabilizer
   120,   300,  -250,     5;   % Wing Left
   120,   300,   250,     5;   % Wing Right
    42,   310,     0,    50;   % Flight Controller (Pixhawk 6C)
    35,   310,     0,    70;   % Power Distribution Board
    22,   350,     0,   -25;   % Telemetry Module (915 MHz)
   530,   290,     0,    60;   % Battery (6S 5000 mAh)
    40,   290,     0,    80;   % Battery Tray
    85,    50,     0,    40;   % Camera / Payload Bay
    65,   325,     0,   120;   % Landing Gear (pair)
    50,   325,     0,    90;   % Bottom Cover
    80,   280,     0,    65;   % ESC x4 (20g each)
    12,   330,     0,    45;   % RC Receiver
    18,   340,     0,   -30;   % GPS Module
    30,   310,     0,    55;   % Wiring Harness
    25,   325,     0,    50;   % Fasteners and Miscellaneous
];

mass_g = data(:,1);
pos_x = data(:,2);
pos_y = data(:,3);
pos_z = data(:,4);

%% Calculate Center of Gravity
total_mass_g = sum(mass_g);
total_mass_kg = total_mass_g / 1000;

cg_x = sum(mass_g .* pos_x) / total_mass_g;
cg_y = sum(mass_g .* pos_y) / total_mass_g;
cg_z = sum(mass_g .* pos_z) / total_mass_g;

fprintf('  Weight Analysis Results:\n');
fprintf('    Total vehicle mass: %.1f g (%.3f kg)\n', total_mass_g, total_mass_kg);
fprintf('    CG location (from nose): X=%.1f mm, Y=%.1f mm, Z=%.1f mm\n', cg_x, cg_y, cg_z);
fprintf('    Payload margin to MTOW: %.1f g\n', 2500 - total_mass_g);

%% Group components by category for the pie chart
categories = {'Propulsion', 'Airframe', 'Avionics', 'Power', 'Payload/Other'};
cat_mass = zeros(1, 5);

% Propulsion: motors, propellers, ESCs, tilt mechanisms
cat_mass(1) = sum(mass_g([1:12, 25]));
% Airframe: fuselage, stabilizer, wings, landing gear, bottom cover, fasteners
cat_mass(2) = sum(mass_g([13, 14, 15, 16, 23, 24, 29]));
% Avionics: FC, telemetry, RC receiver, GPS, wiring
cat_mass(3) = sum(mass_g([17, 19, 26, 27, 28]));
% Power: battery, PDB, battery tray
cat_mass(4) = sum(mass_g([18, 20, 21]));
% Payload: camera/payload bay
cat_mass(5) = sum(mass_g(22));

%% Figure 1: Weight Breakdown Pie Chart
fig1 = figure('Position', [100 100 900 500], 'Color', 'w');

subplot(1,2,1);
labels_pie = cell(1,5);
for k = 1:5
    labels_pie{k} = sprintf('%s\n%.0f g (%.1f%%)', ...
        categories{k}, cat_mass(k), 100*cat_mass(k)/total_mass_g);
end
pie(cat_mass, labels_pie);
title('REI Drone - Mass Distribution by Category', 'FontSize', 13, 'FontWeight', 'bold');
colormap(gca, [0.2 0.6 0.9; 0.3 0.8 0.4; 0.9 0.5 0.2; 0.8 0.2 0.3; 0.6 0.4 0.8]);

subplot(1,2,2);
barh(mass_g, 'FaceColor', [0.2 0.5 0.8]);
set(gca, 'YTick', 1:length(components.name), 'YTickLabel', components.name, 'FontSize', 7);
xlabel('Mass (g)', 'FontSize', 11);
title('Component-Level Mass Breakdown', 'FontSize', 13, 'FontWeight', 'bold');
grid on;
xlim([0 max(mass_g)*1.15]);

sgtitle('REI Drone - Weight and CG Analysis', 'FontSize', 15, 'FontWeight', 'bold');
saveas(fig1, fullfile(figDir, 'weight_cg_analysis.png'));

%% Figure 2: CG Location Visualization (Top View)
fig2 = figure('Position', [100 100 800 600], 'Color', 'w');

% Plot component positions (top view: x vs y)
scatter(pos_x, pos_y, mass_g * 0.5 + 10, mass_g, 'filled', 'MarkerEdgeColor', 'k');
hold on;
plot(cg_x, cg_y, 'rp', 'MarkerSize', 20, 'MarkerFaceColor', 'r', 'LineWidth', 2);
text(cg_x + 15, cg_y + 15, sprintf('CG (%.0f, %.0f) mm', cg_x, cg_y), ...
    'FontSize', 11, 'FontWeight', 'bold', 'Color', 'r');

% Draw simplified airframe outline
fuselage_x = [0, 100, 150, 500, 600, 650, 600, 500, 150, 100, 0];
fuselage_y = [-40, -50, -55, -55, -40, 0, 40, 55, 55, 50, 40];
plot(fuselage_x, fuselage_y, 'k-', 'LineWidth', 1.5);

% Draw wings
plot([200 400], [-500 -500], 'b-', 'LineWidth', 3);
plot([200 400], [500 500], 'b-', 'LineWidth', 3);
plot([200 200], [-100 -500], 'b-', 'LineWidth', 1.5);
plot([400 400], [-100 -500], 'b-', 'LineWidth', 1.5);
plot([200 200], [100 500], 'b-', 'LineWidth', 1.5);
plot([400 400], [100 500], 'b-', 'LineWidth', 1.5);

colorbar;
xlabel('X Position - Forward (mm)', 'FontSize', 12);
ylabel('Y Position - Starboard (mm)', 'FontSize', 12);
title('REI Drone - Component Layout and CG Location (Top View)', 'FontSize', 14, 'FontWeight', 'bold');
grid on; axis equal;
legend('Components (size = mass)', 'Center of Gravity', 'Location', 'best');
hold off;

saveas(fig2, fullfile(figDir, 'cg_location_topview.png'));

%% Store results for other modules
drone_params.total_mass_kg = total_mass_kg;
drone_params.cg = [cg_x, cg_y, cg_z];
drone_params.wingspan_m = 1.0;
drone_params.length_m = 0.65;
drone_params.height_m = 0.18;
drone_params.wing_area_m2 = 0.12;
drone_params.wing_chord_m = 0.12;
drone_params.AR = drone_params.wingspan_m^2 / drone_params.wing_area_m2;
drone_params.num_motors = 4;
drone_params.prop_diameter_m = 0.254;
drone_params.motor_kv = 880;
drone_params.battery_cells = 6;
drone_params.battery_capacity_mah = 5000;
drone_params.battery_voltage_nom = 22.2;

assignin('base', 'drone_params', drone_params);
