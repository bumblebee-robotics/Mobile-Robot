% =========================================================================
% MCT333/MCT344: Mechatronics Omni-Challenge Battery Sizing Model
% Module: Mobile Base & Perception (Duty-Cycle Transient Model)
% =========================================================================
clear; clc; close all;

%% 1. Electrical Rail Definitions & Efficiencies
V_sys  = 12.0;     % Main system operating voltage
V_12V  = 12.0;     % Motor domain
V_5V   = 5.0;      % Processing domain
V_3V3  = 3.3;      % Logic domain

% Converter efficiencies
eta_5V    = 0.90;  % 12V -> 5V buck
eta_3V3   = 0.85;  % 5V -> 3.3V regulator
eta_motor = 1.00;  % Direct battery to motor rail

%% 2. Component Power Definition
comp_names = {'Omni Motors', 'Motor Drivers', 'Raspberry Pi', 'RealSense Cam', ...
              'Encoders', 'CAN Transceiver', 'LED Indicators', 'ESP32 MCU', ...
              'IMU', 'Opto-isolators', 'Buttons/Switches'};

% Format: [Voltage, Quantity, Unit_Inom, Unit_Imax]
components = [
    % --- 12V Rail ---
    12.0,  4,0.800, 5.500;   % Motors
    12.0, 2, 0.030, 0.100;   % Motor Drivers (logic side)
    % --- 5V Rail ---
    5.0,  1, 0.800, 3.000;   % Raspberry Pi
    5.0,  1, 0.700, 1.200;   % RealSense
    5.0,  4, 0.010, 0.015;   % Encoders
    5.0,  1, 0.010, 0.075;   % CAN
    5.0,  5, 0.020, 0.020;   % LEDs
    % --- 3.3V Rail ---
    3.3,  1, 0.080, 0.260;   % ESP32
    3.3,  1, 0.004, 0.005;   % IMU
    3.3,  4, 0.015, 0.020;   % Opto-isolators
    3.3,  3, 0.001, 0.002;   % Buttons
];

V_comp   = components(:,1);
Qty      = components(:,2);
I_nom_u  = components(:,3);
I_max_u  = components(:,4);

I_nom_tot = Qty .* I_nom_u;
I_max_tot = Qty .* I_max_u;

%% 3. Convert Loads to Power (W)
P_comp_nom = V_comp .* I_nom_tot;
P_comp_peak = V_comp .* I_max_tot;

P_12_nom = sum(P_comp_nom(V_comp == 12.0));
P_5_nom  = sum(P_comp_nom(V_comp == 5.0));
P_3_nom  = sum(P_comp_nom(V_comp == 3.3));

P_12_peak = sum(P_comp_peak(V_comp == 12.0));
P_5_peak  = sum(P_comp_peak(V_comp == 5.0));
P_3_peak  = sum(P_comp_peak(V_comp == 3.3));

% Reflect Converter Losses Back to Battery
P_3_input_from_5 = P_3_nom / eta_3V3;
P_3_peak_input   = P_3_peak / eta_3V3;

P_5_total_nom  = (P_5_nom + P_3_input_from_5) / eta_5V;
P_5_total_peak = (P_5_peak + P_3_peak_input) / eta_5V;

P_batt_nom  = P_12_nom + P_5_total_nom;
P_batt_peak = P_12_peak + P_5_total_peak;

%% 4. Battery Pack Definition (Selected Cell)
V_cell = 3.6;
C_cell = 3.0;      % Ah
I_cell_max = 15.0; % A
W_cell = 0.048;    % kg
DoD = 0.80;

I_batt_nom  = P_batt_nom / V_sys;
I_batt_peak = P_batt_peak / V_sys;

%% 5. Mission Profile
T_total = 300;
dt = 0.1;
t = 0:dt:T_total;
P_profile = zeros(size(t));

% Extract Base Logic Power (Total Nominal Power MINUS the 4 Motors)
P_logic_base = P_batt_nom - (4 * 0.800 * 12.0);

% Define Tiered Motor States
I_motor_drive = 0.8;  % Average steady driving
I_motor_agg   = 2.5;  % Aggressive strafing / heavy acceleration
I_motor_stall = 5.5;  % Hard physical stall against a wall/object

% Define Explicit Operational Power States
P_idle   = P_logic_base;                                         % Motors OFF
P_percep = P_logic_base + 12.0;                                  % Motors OFF, Pi/Cam spike (+12W)
P_drive  = P_logic_base + (4 * I_motor_drive * 12.0);            % Nominal driving
P_agg    = P_logic_base + (4 * I_motor_agg * 12.0);              % Aggressive maneuvering
P_stall  = P_logic_base + (4 * I_motor_stall * 12.0);            % TRANSIENT ONLY

% Format: [Start_Time (s), End_Time (s), Power_Draw (W)]
mission_phases = [
      0,  10, P_idle;     % Boot & setup
     10,  65, P_drive;    % Smooth teleop driving
     65,  70, P_agg;      % Aggressive maneuvering near ball pit
     70,  80, P_percep;   % Motors OFF, Perception spike for QR read
     80, 108, P_drive;    % Autonomous driving to drop-off
    108, 110, P_agg;      % Precision Pure X/Y alignment
    110, 111, P_stall;    % TRANSIENT (1s): Bumped the drop-off station
    111, 115, P_agg;      % Correction maneuvering
    115, 130, P_idle;     % Arm working (base motors OFF)
    130, 298, P_drive;    % Driving to exit
    298, 300, P_stall;    % TRANSIENT (2s): Final aggressive stop
];

for k = 1:size(mission_phases,1)
    idx = (t >= mission_phases(k,1)) & (t < mission_phases(k,2));
    P_profile(idx) = mission_phases(k,3);
end
P_profile(end) = mission_phases(end,3);

% Convert Power to Current for Battery Sizing
I_profile = P_profile / V_sys;

%% 6. Energy Integration & Capacity Calculation
Total_As = trapz(t, I_profile); 
C_consumed_Ah = Total_As / 3600;
C_req_Ah = C_consumed_Ah / DoD;

% Calculate Pack Configuration
Ns = ceil(V_sys / V_cell);
Np_cap = ceil(C_req_Ah / C_cell);
Np_curr = ceil(I_batt_peak / I_cell_max);
Np = max(Np_cap, Np_curr);

%% 7. Final Outputs & Weight Check
Total_cells = Ns * Np;
Pack_weight_kg = Total_cells * W_cell;
Total_Pack_Weight = Pack_weight_kg * 1.15; % 15% overhead

Total_Energy_Wh = C_consumed_Ah * V_sys;
Pack_Energy_Wh = (Np * C_cell) * V_sys;
Max_Run_Time_mins = ((Np * C_cell * DoD) / I_batt_nom) * 60;

fprintf('\n=== SYSTEM POWER ARCHITECTURE RESULTS ===\n');
fprintf('Battery Drain (Nominal):  %.2f W (%.2f A)\n', P_batt_nom, I_batt_nom);
fprintf('Battery Drain (Peak):     %.2f W (%.2f A)\n', P_batt_peak, I_batt_peak);
fprintf('-----------------------------------------\n');
fprintf('Mission Energy Consumed:  %.2f Ah (%.2f Wh)\n', C_consumed_Ah, Total_Energy_Wh);
fprintf('Safe Pack Requirement:    %.2f Ah\n', C_req_Ah);
fprintf('-----------------------------------------\n');
fprintf('FINAL CONFIGURATION:      %dS%dP\n', Ns, Np);
fprintf('Installed Pack Energy:    %.2f Ah (%.2f Wh)\n', Np * C_cell, Pack_Energy_Wh);
fprintf('Max Safe Continuous Draw: %.1f A\n', Np * I_cell_max);
fprintf('Estimated Max Run Time:   %.1f Minutes\n', Max_Run_Time_mins);
fprintf('Estimated Pack Weight:    %.2f kg\n', Total_Pack_Weight);
fprintf('=========================================\n');

%% 8. Phase 1 Safety: Main Fuse Sizing
fuse_margin = 1.2; 
I_fuse_calc = I_batt_peak * fuse_margin;
standard_fuses = [5, 7.5, 10, 15, 20, 25, 30, 40, 50, 60, 80];

valid_fuses = standard_fuses(standard_fuses >= I_fuse_calc);
if isempty(valid_fuses)
    Main_Fuse_A = max(standard_fuses);
    warning('Exceeds standard ATC blade fuses.');
else
    Main_Fuse_A = min(valid_fuses);
end

fprintf('\n=== PHASE 1 SAFETY: MAIN FUSE SELECTION ===\n');
fprintf('System Peak Current:      %.2f A\n', I_batt_peak);
fprintf('Calculated Min Fuse:      %.2f A (with 20%% margin)\n', I_fuse_calc);
fprintf('Recommended Main Fuse:    %d A \n', Main_Fuse_A);
fprintf('===========================================\n\n');

%% 9. Advanced Visualizations
% --- Figure 1: Instantaneous Current Profile ---
figure('Name', 'System Current Profile', 'Position', [100, 100, 800, 350]);
plot(t, I_profile, 'b-', 'LineWidth', 1.5); hold on;
fill([t, fliplr(t)], [I_profile, zeros(1, length(I_profile))], 'b', 'FaceAlpha', 0.1, 'EdgeColor', 'none');
yline(I_batt_peak, 'r--', 'Calculated Peak Capability', 'LineWidth', 2, 'LabelHorizontalAlignment', 'left');
yline(I_batt_nom, 'g--', 'Nominal Baseline', 'LineWidth', 2, 'LabelHorizontalAlignment', 'left');
title('Battery Current Drain Over 5-Minute Run (Tiered Motor Logic)');
xlabel('Time (seconds)'); ylabel('Current (Amperes)');
grid on; xlim([0 T_total]); ylim([0 I_batt_peak + 5]);

% --- Figure 2: Cumulative Energy Drain ---
Cumulative_As = cumtrapz(t, I_profile); 
Cumulative_Ah = Cumulative_As / 3600; 
figure('Name', 'Cumulative Capacity', 'Position', [150, 150, 800, 350]);
plot(t, Cumulative_Ah, 'm-', 'LineWidth', 2); hold on;
yline(C_req_Ah, 'k--', 'Required Safe Pack Capacity', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
title('Cumulative Capacity Consumption');
xlabel('Time (seconds)'); ylabel('Capacity Used (Ah)');
grid on; xlim([0 T_total]);

% --- Figure 3: Component Power Breakdown (Watts) ---
figure('Name', 'Power Breakdown Analysis (Watts)', 'Position', [200, 200, 900, 500]);
subplot(1,2,1);
bar(P_comp_nom, 'FaceColor', [0.2 0.6 0.5]);
set(gca, 'xtick', 1:length(comp_names), 'xticklabel', comp_names); xtickangle(45);
title('Nominal Power Breakdown (W)'); ylabel('Continuous Power (Watts)'); grid on;

subplot(1,2,2);
bar(P_comp_peak, 'FaceColor', [0.8 0.3 0.3]);
set(gca, 'xtick', 1:length(comp_names), 'xticklabel', comp_names); xtickangle(45);
title('Peak/Stall Power Breakdown (W)'); ylabel('Maximum Power (Watts)'); grid on;

% --- Figure 4: Subsystem Distribution Pie Chart ---
Actuation_W  = sum(P_comp_nom(1:2));   
Perception_W = sum(P_comp_nom(3:5)) + P_comp_nom(9); 
Control_W    = sum(P_comp_nom(6:8)) + sum(P_comp_nom(10:11)); 

subsystem_powers = [Actuation_W, Perception_W, Control_W];
subsystem_labels = {'Actuation (Drive)', 'Perception & Navigation', 'Control, Comms & UI'};

figure('Name', 'System Power Distribution', 'Position', [300, 300, 600, 450]);
pie(subsystem_powers);
legend(subsystem_labels, 'Location', 'bestoutside');
title('Nominal Power Distribution by Subsystem (Watts)');
colormap(lines(3));