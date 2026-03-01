% =========================================================================
% MCT333/MCT344: Mechatronics Omni-Challenge Battery Sizing Model
% Module: Mobile Base & Perception (Mission-Driven Sizing)
% =========================================================================
clear; clc; close all;
%% 1. Electrical Rail Definitions & Efficiencies
V_sys  = 12.0;     % Base component operating voltage
V_12V  = 12.0;     % Motor domain
V_5V   = 5.0;      % Processing domain
V_3V3  = 3.3;      % Logic domain
% Converter efficiencies
eta_5V    = 0.90;  % 12V -> 5V buck
eta_3V3   = 0.70;  % 5V -> 3.3V regulator
eta_motor = 1.00;  % Direct battery to motor rail
%% 2. Component Power Definition
% Component Labels for Graphing (Strictly aligned with the matrix below)
comp_names = {'12V Geared Motors', 'Raspberry Pi 4B', 'L298N Logic', 'IR Sensors', ...
              'MCP2515 CAN', 'ESP32 MCU', 'Pi Camera V2', 'MPU6050 IMU', ...
              'SN65HVD230 CAN'};
% Format: [Voltage, Quantity, Unit_Inom (A), Unit_Imax (A)]
components = [
    % --- 12V Rail ---
    12.0,  4, 0.800, 4.000;  % 1. 12V Geared Motors
    
    % --- 5V Rail ---
    5.0,   1, 1.500, 3.000;  % 2. Raspberry Pi 4B (4GB)
    5,   1, 0.150, 0.250;  % 6. ESP32 38-Pin Dev Module
    5.0,   2, 0.036, 0.036;  % 3. L298N Logic
    5.0,   4, 0.020, 0.020;  % 4. IR Obstacle Sensors
    
    % --- 3.3V Rail ---
    3.3,   1, 0.250, 0.250;  % 7. Raspberry Pi Camera V2
    3.3,   1, 0.004, 0.005;  % 8. MPU6050 Module
    3.3,   1, 0.015, 0.020;   % 9. SN65HVD230 CAN Tx/Rx
    3.3,   1, 0.005, 0.010;  % 5. MCP2515 CAN Controller
];
V_comp   = components(:,1);
Qty      = components(:,2);
I_nom_u  = components(:,3);
I_max_u  = components(:,4);
I_nom_tot = Qty .* I_nom_u;
I_max_tot = Qty .* I_max_u;
%% 3. Convert Loads to Power (W)
P_comp_nom  = V_comp .* I_nom_tot;
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
% Absolute theoretical maximums (Hardware Limits)
P_batt_nom  = P_12_nom + P_5_total_nom;
P_batt_peak = P_12_peak + P_5_total_peak;
%% 4. Battery Pack Definition (Selected Cell)
V_cell = 3.6;      % Nominal Li-ion cell voltage
C_cell = 3.0;      % Ah
I_cell_max = 15.0; % A
W_cell = 0.048;    % kg
DoD = 0.80;
I_batt_nom  = P_batt_nom / V_sys;
I_batt_peak = P_batt_peak / V_sys; % Theoretical hardware limit (Not for sizing)
%% 5. Finite State Machine Mission Model (Optimized & Exact Timing)
T_limit = 300;   % Total mission time (s)
dt = 0.1;        % Simulation timestep
t = 0:dt:T_limit; % Exact time vector (guarantees exactly 300.0s)
% Preallocate arrays for massive performance boost
P_profile = zeros(size(t));
state_log = zeros(size(t));
% Base logic power (exclude motors)
P_logic_base = P_batt_nom - (4 * 0.8 * 12);
% Motor current levels (A)
I_motor_drive  = 0.8;   % nominal driving
I_motor_agg    = 2.5;   % aggressive maneuver / strafing
I_motor_stall  = 4;   % stall spike
% Nominal power for FSM operations (W)
P_idle   = P_logic_base;
P_percep = P_logic_base + 12;    % Pi & Camera vision spike
P_pick   = P_logic_base;         % Arm handles pick, base idles
P_drop   = P_logic_base;         % Arm handles drop, base idles
% Define FSM States
STATE.BOOT        = 0;
STATE.MANUAL      = 1;
STATE.ALIGN       = 2;
STATE.PICK        = 3;
STATE.QR_READ     = 4;
STATE.TRANSPORT   = 5;
STATE.CONSTRAINED = 6;
STATE.DROP        = 7;
STATE.TRANSIT     = 8;
STATE.EXIT        = 9;
STATE.FINISHED    = 10;
% --- Mission Duration States (Mathematically sums to exactly 300s) ---
Dur.BOOT          = 5;    % System boot
Dur.MANUAL        = 70;   % Maze teleop
Dur.ALIGN         = 12;   % Vision alignment (longer for accuracy)
Dur.PICK          = 10;   % Arm kinematics (careful picking)
Dur.QR_READ       = 3;    % Pi 4 inference spike
Dur.TRANSPORT     = 25;   % Drive to drop zones
Dur.CONSTRAINED   = 14;   % Pure X/Y strafing (heavy current)
Dur.DROP          = 7;    % Releasing gripper
Dur.TRANSIT       = 10;   % Drive between Red/Blue/Green zones
Dur.EXIT          = 42;   % Final stretch to exit
Dur.STALL_SPIKE   = 1.5;  % Transient spike at end of maneuvers
N_boxes_target = 3;
% Initialize FSM variables
current_state = STATE.BOOT;
state_timer = 0;
cube_counter = 0;       
drop_counter = 0;       
% High-Performance Simulation Loop
for i = 1:length(t)
    % 1. Determine motor current based on FSM State
    I_motor = 0;
    if current_state == STATE.MANUAL || current_state == STATE.TRANSPORT || current_state == STATE.TRANSIT || current_state == STATE.EXIT
        I_motor = I_motor_drive;
    elseif current_state == STATE.ALIGN 
        I_motor = I_motor_agg;
        % Transient stall at the end of alignment
        if state_timer >= (Dur.ALIGN - Dur.STALL_SPIKE), I_motor = I_motor_stall; end
    elseif current_state == STATE.CONSTRAINED
        I_motor = I_motor_agg;
        % Transient stall at the end of strafing
        if state_timer >= (Dur.CONSTRAINED - Dur.STALL_SPIKE), I_motor = I_motor_stall; end
    end
    
    % 2. Compute instantaneous power
    switch current_state
        case STATE.BOOT,     P_current = P_idle;
        case STATE.PICK,     P_current = P_pick;
        case STATE.QR_READ,  P_current = P_percep;
        case STATE.DROP,     P_current = P_drop;
        case STATE.FINISHED, P_current = P_idle; 
        otherwise,           P_current = P_logic_base + 4 * 12 * I_motor;
    end
    
    % Log data
    P_profile(i) = P_current;
    state_log(i) = current_state;
    
    % 3. State Machine Transition Logic
    state_timer = round(state_timer + dt, 3); 
    
    switch current_state
        case STATE.BOOT
            if state_timer >= Dur.BOOT, current_state = STATE.MANUAL; state_timer = 0; end
        case STATE.MANUAL
            if state_timer >= Dur.MANUAL, current_state = STATE.ALIGN; state_timer = 0; end
        case STATE.ALIGN
            if state_timer >= Dur.ALIGN, current_state = STATE.PICK; state_timer = 0; end
        case STATE.PICK
            if state_timer >= Dur.PICK
                cube_counter = cube_counter + 1;
                current_state = STATE.QR_READ; state_timer = 0;
            end
        case STATE.QR_READ
            if state_timer >= Dur.QR_READ
                if cube_counter < N_boxes_target, current_state = STATE.ALIGN; state_timer = 0; 
                else, current_state = STATE.TRANSPORT; state_timer = 0; end
            end
        case STATE.TRANSPORT
            if state_timer >= Dur.TRANSPORT, current_state = STATE.CONSTRAINED; state_timer = 0; end
        case STATE.CONSTRAINED
            if state_timer >= Dur.CONSTRAINED, current_state = STATE.DROP; state_timer = 0; end
        case STATE.DROP
            if state_timer >= Dur.DROP
                drop_counter = drop_counter + 1;
                if drop_counter < N_boxes_target, current_state = STATE.TRANSIT; state_timer = 0; 
                else, current_state = STATE.EXIT; state_timer = 0; end
            end
        case STATE.TRANSIT
            if state_timer >= Dur.TRANSIT, current_state = STATE.CONSTRAINED; state_timer = 0; end
        case STATE.EXIT
            if state_timer >= Dur.EXIT, current_state = STATE.FINISHED; state_timer = 0; end
        case STATE.FINISHED
            % Idle until loop finishes
    end
end
% Extract Mission-Based Peak (NOT Theoretical Peak)
I_profile = P_profile / V_sys;
I_mission_peak = max(I_profile); 
fprintf('\n=== FSM MISSION SUMMARY ===\n');
fprintf('Total Simulation Time: %.1f sec\n', t(end));
fprintf('Cubes Picked: %d / %d\n', cube_counter, N_boxes_target);
fprintf('Cubes Dropped: %d / %d\n', drop_counter, N_boxes_target);
fprintf('Actual FSM Mission Peak: %.2f A (vs Theoretical: %.2f A)\n', I_mission_peak, I_batt_peak);
fprintf('===================================================\n');
%% 6. Energy Integration & Capacity Calculation
Total_As = trapz(t, I_profile); 
C_consumed_Ah = Total_As / 3600;
C_req_Ah = C_consumed_Ah / DoD;
% Calculate Pack Configuration using FSM MISSION PEAK
Ns = ceil(V_sys / V_cell);
Np_cap = ceil(C_req_Ah / C_cell);
Np_curr = ceil(I_mission_peak / I_cell_max); % Corrected logic
Np = max(Np_cap, Np_curr);
%% 7. Final Outputs & Architectural Notes
Total_cells = Ns * Np;
Pack_weight_kg = Total_cells * W_cell;
Total_Pack_Weight = Pack_weight_kg * 1.15; % 15% overhead
Total_Energy_Wh = C_consumed_Ah * V_sys;
Pack_Energy_Wh = (Np * C_cell) * V_sys;
% Mission average power (true average over FSM profile)
P_avg_mission = mean(P_profile);        % Watts
% Total usable pack energy (Wh)
Usable_Pack_Energy_Wh = Pack_Energy_Wh * DoD;
% Runtime based on energy balance
Max_Run_Time_hours = Usable_Pack_Energy_Wh / P_avg_mission;
Max_Run_Time_mins  = Max_Run_Time_hours * 60;
%Max_Run_Time_mins = ((Np * C_cell * DoD) / I_batt_nom) * 60; % Inom static
fprintf('\n=== SYSTEM POWER ARCHITECTURE RESULTS ===\n');
fprintf('Mission Energy Consumed:  %.2f Ah (%.2f Wh)\n', C_consumed_Ah, Total_Energy_Wh);
fprintf('Safe Pack Requirement:    %.2f Ah\n', C_req_Ah);
fprintf('-----------------------------------------\n');
fprintf('FINAL CONFIGURATION:      %dS%dP\n', Ns, Np);
fprintf('Installed Pack Energy:    %.2f Ah (%.2f Wh)\n', Np * C_cell, Pack_Energy_Wh);
fprintf('Max Safe Continuous Draw: %.1f A\n', Np * I_cell_max);
fprintf('Estimated Max Run Time:   %.1f Minutes\n', Max_Run_Time_mins);
fprintf('Estimated Pack Weight:    %.2f kg\n', Total_Pack_Weight);
fprintf('-----------------------------------------\n');
fprintf('*** VOLTAGE ARCHITECTURE & SAFETY WARNING ***\n');
fprintf('Nominal Pack Voltage:     %.1f V\n', Ns * V_cell);
fprintf('Max Charge Voltage:       %.1f V\n', Ns * 4.2);
fprintf('=========================================\n');
%% 8. Phase 1 Safety: Main Fuse Sizing
% The fuse is sized based on the actual FSM Mission Peak, plus a safety margin.
fuse_margin = 1.25; 
I_fuse_calc = I_mission_peak * fuse_margin;
standard_fuses = [5, 7.5, 10, 15, 20, 25, 30, 40, 50, 60, 80];
valid_fuses = standard_fuses(standard_fuses >= I_fuse_calc);
if isempty(valid_fuses)
    Main_Fuse_A = max(standard_fuses);
    warning('Exceeds standard ATC blade fuses.');
else
    Main_Fuse_A = min(valid_fuses);
end
fprintf('\n=== PHASE 1 SAFETY: MAIN FUSE SELECTION ===\n');
fprintf('Mission Peak Current:     %.2f A\n', I_mission_peak);
fprintf('Calculated Min Fuse:      %.2f A (with 25%% margin)\n', I_fuse_calc);
fprintf('Recommended Main Fuse:    %d A \n', Main_Fuse_A);
fprintf('===========================================\n\n');
%% 9. Advanced Visualizations
% --- Figure 1: Instantaneous Current Profile ---
figure('Name', 'System Current Profile', 'Position', [100, 100, 800, 350]);
plot(t, I_profile, 'b-', 'LineWidth', 1.5); hold on;
fill([t, fliplr(t)], [I_profile, zeros(1, length(I_profile))], 'b', 'FaceAlpha', 0.1, 'EdgeColor', 'none');
yline(I_mission_peak, 'r--', 'Mission Peak (Sizing Baseline)', 'LineWidth', 2, 'LabelHorizontalAlignment', 'left');
yline(I_batt_nom, 'g--', 'Nominal Baseline', 'LineWidth', 2, 'LabelHorizontalAlignment', 'left');
title('Battery Current Drain Over 5-Minute Run (Tiered FSM)');
xlabel('Time (seconds)'); ylabel('Current (Amperes)');
grid on; xlim([0 T_limit]); ylim([0 I_mission_peak + 5]);
% --- Figure 2: Cumulative Energy Drain ---
Cumulative_As = cumtrapz(t, I_profile); 
Cumulative_Ah = Cumulative_As / 3600; 
figure('Name', 'Cumulative Capacity', 'Position', [150, 150, 800, 350]);
plot(t, Cumulative_Ah, 'm-', 'LineWidth', 2); hold on;
yline(C_req_Ah, 'k--', 'Required Safe Pack Capacity', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
title('Cumulative Capacity Consumption');
xlabel('Time (seconds)'); ylabel('Capacity Used (Ah)');
grid on; xlim([0 T_limit]);
% --- Figure 3: Component Power Breakdown (Watts) ---
figure('Name', 'Power Breakdown Analysis (Watts)', 'Position', [200, 200, 900, 500]);
subplot(1,2,1);
bar(P_comp_nom, 'FaceColor', [0.2 0.6 0.5]);
set(gca, 'xtick', 1:length(comp_names), 'xticklabel', comp_names); xtickangle(45);
title('Nominal Power Breakdown (W)'); ylabel('Continuous Power (Watts)'); grid on;
subplot(1,2,2);
bar(P_comp_peak, 'FaceColor', [0.8 0.3 0.3]);
set(gca, 'xtick', 1:length(comp_names), 'xticklabel', comp_names); xtickangle(45);
title('Theoretical Peak Power Breakdown (W)'); ylabel('Maximum Power (Watts)'); grid on;