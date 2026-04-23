%% REI Drone - Telemetry RF Link Budget Analysis
%  Rootcastle Engineering & Innovation
%
%  Analyzes the 915 MHz telemetry link between the REI Drone and the
%  ground control station. Computes free-space path loss, link margin,
%  and maximum reliable range under various conditions.
%
%  Author: Batuhan Ayribas, M.Sc.

figDir = getappdata(0, 'figDir');
if isempty(figDir)
    figDir = fullfile(fileparts(mfilename('fullpath')), '..', 'figures');
    if ~exist(figDir, 'dir'), mkdir(figDir); end
end

%% Link Parameters
f = 915e6;                          % Frequency (Hz)
c = 3e8;                            % Speed of light (m/s)
lambda = c / f;                     % Wavelength (m)

% Transmitter (on drone)
P_tx_dBm = 20;                     % Transmit power (dBm) = 100 mW
G_tx_dBi = 2;                      % Antenna gain (dBi) - omnidirectional
L_tx_cable = 0.5;                   % Cable/connector loss (dB)

% Receiver (ground station)
G_rx_dBi = 5;                      % Antenna gain (dBi) - directional
L_rx_cable = 0.5;                   % Cable/connector loss (dB)
NF_rx = 3;                         % Receiver noise figure (dB)
BW = 200e3;                        % Bandwidth (Hz)
SNR_required = 10;                  % Required SNR for reliable link (dB)
L_misc = 3;                        % Miscellaneous losses (dB): polarization, body

% Receiver sensitivity
kT = -174;                         % Thermal noise floor (dBm/Hz)
N_floor = kT + 10*log10(BW) + NF_rx;  % Noise floor (dBm)
Sensitivity = N_floor + SNR_required;   % Receiver sensitivity (dBm)

%% Distance Analysis
d = logspace(1, 5, 500);           % Distance: 10 m to 100 km

% Free-space path loss (FSPL)
FSPL = 20*log10(d) + 20*log10(f) - 147.55;

% Received power
P_rx = P_tx_dBm + G_tx_dBi + G_rx_dBi - L_tx_cable - L_rx_cable - L_misc - FSPL;

% Link margin
link_margin = P_rx - Sensitivity;

% Maximum range (where link margin = 0)
idx_max = find(link_margin <= 0, 1, 'first');
if ~isempty(idx_max)
    d_max = d(idx_max);
else
    d_max = d(end);
end

% Practical range (6 dB margin for fading)
idx_practical = find(link_margin <= 6, 1, 'first');
if ~isempty(idx_practical)
    d_practical = d(idx_practical);
else
    d_practical = d(end);
end

fprintf('  Telemetry Link Budget:\n');
fprintf('    Frequency: %.0f MHz\n', f/1e6);
fprintf('    TX Power: %d dBm (%.0f mW)\n', P_tx_dBm, 10^(P_tx_dBm/10));
fprintf('    Receiver Sensitivity: %.1f dBm\n', Sensitivity);
fprintf('    Max theoretical range: %.1f km\n', d_max/1000);
fprintf('    Practical range (6dB margin): %.1f km\n', d_practical/1000);

%% Figure 1: Link Budget Analysis
fig1 = figure('Position', [100 100 1200 500], 'Color', 'w');

subplot(1,3,1);
semilogx(d/1000, FSPL, 'b-', 'LineWidth', 2);
xlabel('Distance (km)'); ylabel('Path Loss (dB)');
title('Free-Space Path Loss'); grid on;

subplot(1,3,2);
semilogx(d/1000, P_rx, 'b-', 'LineWidth', 2); hold on;
yline(Sensitivity, 'r--', 'LineWidth', 1.5);
plot(d_max/1000, Sensitivity, 'rp', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
xlabel('Distance (km)'); ylabel('Received Power (dBm)');
title('Received Signal Strength'); grid on;
legend('P_{rx}', 'Sensitivity', sprintf('Max: %.1f km', d_max/1000));

subplot(1,3,3);
semilogx(d/1000, link_margin, 'b-', 'LineWidth', 2); hold on;
yline(0, 'r--', 'LineWidth', 1.5);
yline(6, 'g--', 'LineWidth', 1);
plot(d_max/1000, 0, 'rp', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
plot(d_practical/1000, 6, 'gp', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
xlabel('Distance (km)'); ylabel('Link Margin (dB)');
title('Link Margin'); grid on;
legend('Margin', 'Threshold', '6 dB Fade Margin', 'Location', 'best');

sgtitle('REI Drone - 915 MHz Telemetry Link Budget', 'FontSize', 14, 'FontWeight', 'bold');
saveas(fig1, fullfile(figDir, 'telemetry_link_budget.png'));

%% Figure 2: Range vs TX Power
fig2 = figure('Position', [100 100 600 400], 'Color', 'w');

P_tx_sweep = 10:2:30;  % dBm
d_max_sweep = zeros(size(P_tx_sweep));

for i = 1:length(P_tx_sweep)
    Prx_sweep = P_tx_sweep(i) + G_tx_dBi + G_rx_dBi - L_tx_cable - L_rx_cable - L_misc - FSPL;
    margin_sweep = Prx_sweep - Sensitivity;
    idx = find(margin_sweep <= 6, 1, 'first');
    if ~isempty(idx)
        d_max_sweep(i) = d(idx) / 1000;
    else
        d_max_sweep(i) = d(end) / 1000;
    end
end

plot(P_tx_sweep, d_max_sweep, 'b-o', 'LineWidth', 2, 'MarkerFaceColor', 'b');
hold on;
plot(P_tx_dBm, d_practical/1000, 'rp', 'MarkerSize', 15, 'MarkerFaceColor', 'r');
xlabel('TX Power (dBm)'); ylabel('Practical Range (km)');
title('REI Drone - Range vs Transmit Power (6 dB margin)', 'FontSize', 13, 'FontWeight', 'bold');
grid on;
legend('Range Curve', sprintf('Current: %.0f dBm, %.1f km', P_tx_dBm, d_practical/1000));
saveas(fig2, fullfile(figDir, 'telemetry_range_vs_power.png'));
