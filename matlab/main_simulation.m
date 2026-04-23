%% REI Drone - Master Simulation Runner
%  Rootcastle Engineering & Innovation
%  VTOL Tilt-Rotor UAV - Complete System Simulation
%
%  Author: Batuhan Ayribas, M.Sc.
%  Affiliation: Rootcastle Engineering & Innovation
%  Date: 2026
%
%  This script runs all simulation modules sequentially and generates
%  publication-quality figures for each subsystem analysis. The REI Drone
%  is a modular open-source VTOL tilt-rotor UAV designed for research,
%  mapping, survey, and inspection missions.
%
%  Vehicle Configuration:
%    - VTOL Tilt-Rotor with 4 brushless motors
%    - Wingspan: 1000 mm, Length: 650 mm, Height: 180 mm
%    - Motors: 2216 880KV Brushless
%    - Propellers: 10x4.5 inch (3-blade)
%    - Battery: 6S LiPo, 5000 mAh
%    - Flight Controller: Pixhawk 6C / Cube Orange
%    - Telemetry: 915 MHz, up to 20 km range

clear; clc; close all;

fprintf('==========================================================\n');
fprintf('  REI Drone - VTOL Tilt-Rotor UAV Simulation Suite\n');
fprintf('  Rootcastle Engineering & Innovation\n');
fprintf('  Author: Batuhan Ayribas, M.Sc.\n');
fprintf('==========================================================\n\n');

% Create output directory for figures
figDir = fullfile(fileparts(mfilename('fullpath')), '..', 'figures');
if ~exist(figDir, 'dir')
    mkdir(figDir);
end

% Store figure directory path for all modules
setappdata(0, 'figDir', figDir);

%% Run Simulation Modules
modules = {
    'weight_cg_analysis',      'Weight and CG Analysis';
    'aerodynamics_analysis',   'Aerodynamic Performance';
    'motor_propulsion',        'Motor and Propulsion';
    'battery_endurance',       'Battery Endurance';
    'flight_dynamics',         '6-DOF Flight Dynamics';
    'pid_controller',          'PID Controller Tuning';
    'tilt_transition',         'VTOL-to-Cruise Transition';
    'telemetry_link_budget',   'Telemetry Link Budget';
    'structural_loads',        'Structural Load Analysis';
    'mission_planner',         'Autonomous Mission Planning';
};

totalModules = size(modules, 1);
results = struct();

for i = 1:totalModules
    moduleName = modules{i, 1};
    moduleDesc = modules{i, 2};

    fprintf('[%2d/%2d] Running: %s\n', i, totalModules, moduleDesc);
    fprintf('        Module: %s.m\n', moduleName);

    try
        tic;
        run(moduleName);
        elapsed = toc;
        fprintf('        Status: COMPLETED (%.2f seconds)\n\n', elapsed);
        results.(moduleName) = 'PASS';
    catch ME
        fprintf('        Status: FAILED - %s\n\n', ME.message);
        results.(moduleName) = ['FAIL: ' ME.message];
    end
end

%% Summary Report
fprintf('==========================================================\n');
fprintf('  Simulation Summary\n');
fprintf('==========================================================\n');

fields = fieldnames(results);
passCount = 0;
failCount = 0;
for i = 1:length(fields)
    status = results.(fields{i});
    if strcmp(status, 'PASS')
        passCount = passCount + 1;
        fprintf('  [PASS] %s\n', fields{i});
    else
        failCount = failCount + 1;
        fprintf('  [FAIL] %s: %s\n', fields{i}, status);
    end
end

fprintf('\n  Total: %d modules, %d passed, %d failed\n', ...
    length(fields), passCount, failCount);
fprintf('  Figures saved to: %s\n', figDir);
fprintf('==========================================================\n');
