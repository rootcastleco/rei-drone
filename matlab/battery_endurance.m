%% REI Drone - Battery Endurance Analysis
%  Rootcastle Engineering & Innovation
%
%  Models the 6S LiPo 5000 mAh battery discharge across different flight
%  phases: hover, transition, and cruise. Predicts endurance, voltage sag,
%  and remaining capacity over the mission profile.
%
%  Author: Batuhan Ayribas, M.Sc.

figDir = getappdata(0, 'figDir');
if isempty(figDir)
    figDir = fullfile(fileparts(mfilename('fullpath')), '..', 'figures');
    if ~exist(figDir, 'dir'), mkdir(figDir); end
end

%% Battery Parameters (6S LiPo)
n_cells = 6;
V_cell_full = 4.2;                  % Fully charged cell voltage
V_cell_nom = 3.7;                   % Nominal cell voltage
V_cell_cutoff = 3.5;               % Low voltage cutoff per cell
Q_total = 5.0;                      % Total capacity (Ah)
R_internal = 0.018;                 % Internal resistance per cell (Ohm)
R_total = n_cells * R_internal;     % Total pack resistance

V_full = n_cells * V_cell_full;     % 25.2 V
V_nom = n_cells * V_cell_nom;       % 22.2 V
V_cutoff = n_cells * V_cell_cutoff; % 21.0 V

%% Discharge Model
% Open-circuit voltage as a function of SOC (empirical LiPo curve)
SOC_pts = [1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0.05, 0.0];
V_oc_pts = [4.20, 4.10, 4.02, 3.95, 3.87, 3.82, 3.78, 3.74, 3.70, 3.65, 3.55, 3.30];
V_oc_pts = V_oc_pts * n_cells;     % Scale to pack voltage

%% Flight Phase Current Draws
% Based on propulsion analysis: hover ~298W, cruise ~165W, transition ~240W
I_hover = 298 / V_nom;             % About 13.4 A
I_cruise = 165 / V_nom;            % About 7.4 A
I_transition = 240 / V_nom;        % About 10.8 A

%% Simulation: Full Mission Profile
% Mission: 30s hover -> 5s transition -> cruise until cutoff -> 5s transition -> 30s hover -> land
dt = 0.5;                          % Time step (seconds)
t_max = 3600;                      % Maximum simulation time (1 hour)
t = 0:dt:t_max;
N = length(t);

SOC = zeros(1, N); SOC(1) = 1.0;
V_terminal = zeros(1, N);
I_draw = zeros(1, N);
phase = cell(1, N);
P_consumed = zeros(1, N);
energy_used = zeros(1, N);

mission_ended = false;
land_time = 0;

for k = 1:N
    % Determine flight phase
    if t(k) < 30
        I_draw(k) = I_hover;
        phase{k} = 'Hover (Takeoff)';
    elseif t(k) < 35
        I_draw(k) = I_transition;
        phase{k} = 'Transition (VTOL->Cruise)';
    elseif ~mission_ended
        I_draw(k) = I_cruise;
        phase{k} = 'Cruise';
    else
        if land_time == 0, land_time = t(k); end
        remaining = t(k) - land_time;
        if remaining < 5
            I_draw(k) = I_transition;
            phase{k} = 'Transition (Cruise->VTOL)';
        elseif remaining < 35
            I_draw(k) = I_hover;
            phase{k} = 'Hover (Landing)';
        else
            I_draw(k) = 0;
            phase{k} = 'Landed';
        end
    end

    % Calculate terminal voltage
    V_oc = interp1(SOC_pts, V_oc_pts, SOC(k), 'pchip', V_oc_pts(end));
    V_terminal(k) = V_oc - I_draw(k) * R_total;
    P_consumed(k) = V_terminal(k) * I_draw(k);

    if k > 1
        energy_used(k) = energy_used(k-1) + P_consumed(k) * dt / 3600;
    end

    % Check for low voltage cutoff during cruise
    if V_terminal(k) <= V_cutoff && ~mission_ended && t(k) > 40
        mission_ended = true;
    end

    % Update SOC
    if k < N
        SOC(k+1) = SOC(k) - (I_draw(k) * dt) / (Q_total * 3600);
        SOC(k+1) = max(SOC(k+1), 0);
    end

    % Stop simulation if landed
    if strcmp(phase{k}, 'Landed') && (t(k) - land_time) > 40
        t = t(1:k); SOC = SOC(1:k); V_terminal = V_terminal(1:k);
        I_draw = I_draw(1:k); P_consumed = P_consumed(1:k);
        energy_used = energy_used(1:k); phase = phase(1:k);
        break;
    end
end

total_flight_time_min = t(end) / 60;
total_energy_wh = energy_used(end);
cruise_time = sum(strcmp(phase, 'Cruise')) * dt / 60;

fprintf('  Battery Endurance Results:\n');
fprintf('    Total flight time: %.1f min\n', total_flight_time_min);
fprintf('    Cruise time: %.1f min\n', cruise_time);
fprintf('    Total energy consumed: %.1f Wh\n', total_energy_wh);
fprintf('    Final SOC: %.1f%%\n', SOC(end)*100);

%% Figure 1: Battery Discharge Profile
fig1 = figure('Position', [100 100 1200 800], 'Color', 'w');

subplot(2,2,1);
plot(t/60, V_terminal, 'b-', 'LineWidth', 2); hold on;
yline(V_cutoff, 'r--', 'LineWidth', 1.5);
xlabel('Time (min)'); ylabel('Voltage (V)');
title('Terminal Voltage'); grid on;
legend('V_{terminal}', 'Cutoff Voltage');

subplot(2,2,2);
plot(t/60, SOC*100, 'b-', 'LineWidth', 2);
xlabel('Time (min)'); ylabel('SOC (%)');
title('State of Charge'); grid on; ylim([0 105]);

subplot(2,2,3);
plot(t/60, I_draw, 'b-', 'LineWidth', 2);
xlabel('Time (min)'); ylabel('Current (A)');
title('Discharge Current'); grid on;

subplot(2,2,4);
plot(t/60, P_consumed, 'b-', 'LineWidth', 2);
xlabel('Time (min)'); ylabel('Power (W)');
title('Power Consumption'); grid on;

sgtitle(sprintf('REI Drone - Battery Discharge (6S 5000mAh) - Total: %.1f min', ...
    total_flight_time_min), 'FontSize', 14, 'FontWeight', 'bold');
saveas(fig1, fullfile(figDir, 'battery_endurance.png'));

%% Figure 2: Energy Budget
fig2 = figure('Position', [100 100 800 400], 'Color', 'w');

subplot(1,2,1);
plot(t/60, energy_used, 'b-', 'LineWidth', 2);
xlabel('Time (min)'); ylabel('Energy (Wh)');
title('Cumulative Energy Consumption'); grid on;

subplot(1,2,2);
% Energy by phase
phases_unique = {'Hover (Takeoff)', 'Transition (VTOL->Cruise)', 'Cruise', ...
    'Transition (Cruise->VTOL)', 'Hover (Landing)'};
energy_phase = zeros(1, 5);
for p = 1:5
    mask = strcmp(phase, phases_unique{p});
    energy_phase(p) = sum(P_consumed(mask) * dt / 3600);
end
bar(energy_phase, 'FaceColor', [0.2 0.5 0.8]);
set(gca, 'XTickLabel', {'Hover Up', 'Trans 1', 'Cruise', 'Trans 2', 'Hover Down'}, ...
    'FontSize', 8);
ylabel('Energy (Wh)'); title('Energy by Flight Phase'); grid on;

sgtitle('REI Drone - Energy Budget Analysis', 'FontSize', 14, 'FontWeight', 'bold');
saveas(fig2, fullfile(figDir, 'energy_budget.png'));
