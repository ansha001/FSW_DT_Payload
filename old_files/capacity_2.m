%% Clear and Close
clear all;
%close all;

%% 1) Load Lookup Tables (Unchanged)
load('RC_Rs_values_1.mat'); 
load('RC_Rs_values_1_charging.mat');
load('LIR2032_EEMB_Cell1_25C_OCV.mat');
load('data_Cell2_25C.mat')
SOC_OCV = SOC;

% Extract lookup table values - Discharging params
SOC_table = lookupTable.SOC;
Rs_table  = lookupTable.Rs;
R1_table  = lookupTable.R1;
C1_table  = lookupTable.C1;

% Extract lookup table values - Charging params
SOC_1_table = lookupTable_1.SOC;
Rs_1_table  = lookupTable_1.Rs;
R1_1_table  = lookupTable_1.R1;
C1_1_table  = lookupTable_1.C1;

% Polynomial fit for OCV-charging(SOC)
degree = 5;
coefficients_1 = polyfit(SOC, OCV_charge, degree);
SOC_fit_1      = linspace(0, 1, 100);
OCV_fit_1      = polyval(coefficients_1, SOC_fit_1);

% Polynomial fit for OCV-discharging(SOC)
coefficients   = polyfit(SOC, OCV_discharge, degree);
SOC_fit        = linspace(0, 1, 100);
OCV_fit        = polyval(coefficients, SOC_fit);

% Derivatives of OCV(SOC)
derivative_coefficients_1 = polyder(coefficients_1);
dOCV_dSOC_1 = @(soc) polyval(derivative_coefficients_1, soc);

derivative_coefficients   = polyder(coefficients);
dOCV_dSOC      = @(soc) polyval(derivative_coefficients, soc);

%% 2) Define Cycle Segments and Capacities from Table
%    (Adjust to match your data files and the table you showed)
cycleSets   = {'1_20', '21_40', '41_60', '61_80', '81_100', '101_120','121_140', '141_160', '161_180', '181_200', '201_220', '221_240', '241_260' };

% "Initial" capacities for each 20-cycle chunk (start-of-chunk)
initCap_Ah  = data_Cell2_25C.plot_capacity(1:end-1)./1000;
% "End" capacities for each chunk (target)
endCap_Ah   = data_Cell2_25C.plot_capacity(2:end)./1000;
factor = 1;  % scaling factor if you have one

%% 3) EKF Initializations (Carry Over Across Segments)
% -----------------------------------------------------
% State EKF
P_state = diag([1e-7, 1e-7]);  % Will be updated/carry over
Q_state = diag([1e-6, 1]);  % updated from diag([1e-6, 1e-3])
R_state = 0.015;

% Parameter EKF
P_param = 1e-1;  % Will be updated/carry over
Q_param = 1e-1;

R_param  = 4.1e-4; % 2100
%R_param = 5.2e-4;  % 2100
%R_param = 0.5e-4; % 5200
%R_param = 3.14e-5; % 9000
%R_param = 1e-5;

% update intervals
update_interval = 2100; % capacity parameter update frequency
beta_update_interval = 100000; % beta

% Initial states 
x_hat_state = [1.0; 0];  % [SOC; Vc1] 
x_hat_param = initCap_Ah(1)*3600/factor;  % start with the capacity for the first segment

% Initialize second capacity estimate
second_capacity = x_hat_param;  % Start with the initial primary capacity (in Coulombs)
last_beta_update_time = 0;     % Last time beta was updated (in seconds)
last_capacity = x_hat_param;   % Last capacity value when beta was updated (in Coulombs)
beta_rate = -0.5e-7;                      % Rate of change for second capacity (capacity per second)

%% 4) Prepare Arrays to Accumulate Full Results for All Segments
allTime     = [];
allSOC      = [];
allSOC_true = [];
allV        = [];
allV_true   = [];
allCapEst   = [];
allSecondCap = [];  % Array to store second capacity estimates across all segments
allBetaTimes = [];  % Array to store times when beta is updated
allBetaValues = []; % Array to store beta values

% We also track the start/end times of each segment for plotting horizontal lines
segmentStart = zeros(1, length(cycleSets));
segmentEnd   = zeros(1, length(cycleSets));

timeOffset = 0;  % offset so consecutive segments don't start at t=0
K_state   = [0; 0];
% For parameter EKF
dx_by_dtheta_k   = [0; 0];
dx_by_dtheta_k_1 = [0; 0];


%% 5) Main Loop Over Each 20-Cycle Segment
for segIdx = 1:length(cycleSets)
    
    % (a) Load data for this segment
    dataFile = sprintf('LIR2032_EEMB_Cell2_25C_Aging_Cycles_%s.mat', cycleSets{segIdx});
    profile  = load(dataFile);
    
    dt       = profile.dt;
    time_raw = profile.time;
    time     = dt:dt:time_raw(end);
    
    % Interpolate current and voltage
    I       = interp1(time_raw', -profile.I',      time, 'linear', 'extrap');
    V_true  = interp1(time_raw',  profile.V_true', time, 'linear', 'extrap');
    
    SOC_0   = profile.SOC_0;  % initial true SOC from data
    
    % (b) Update battery capacity info for this segment
    battery_capacity_Ah = initCap_Ah(segIdx);
    battery_capacity_C  = battery_capacity_Ah * 3600 / factor;
    
    target_capacity_Ah  = endCap_Ah(segIdx);
    target_mAh          = target_capacity_Ah * 1000;
    
    % (c) Initialize local arrays for this segment
    SOC_true  = zeros(size(time));
    SOC_true(1) = SOC_0;  % "true" SOC for comparison
    
    SOC_seg    = zeros(size(time));
    Vc1_seg    = zeros(size(time));
    V_seg      = zeros(size(time));
    Rs_seg     = zeros(size(time));
    R1_seg     = zeros(size(time));
    C1_seg     = zeros(size(time));
    OCV_seg    = zeros(size(time));
    capEst_seg = zeros(size(time));
    secondCap_seg = zeros(size(time));  % Array for second capacity in this segment
    
    % Use the last known state estimates as the "initial" for this segment
    x_hat_state(1) = SOC_0;
    SOC_seg(1)     = x_hat_state(1);
    Vc1_seg(1)     = x_hat_state(2);
    capEst_seg(1)  = x_hat_param;
    secondCap_seg(1) = second_capacity;  % Initial second capacity carried over
    
    % Parameter lookups for the very first sample in this segment
    Rs_seg(1)  = interp1(SOC_1_table, Rs_1_table, x_hat_state(1),  "linear","extrap");
    R1_seg(1)  = interp1(SOC_1_table, R1_1_table, x_hat_state(1),  "linear","extrap");
    C1_seg(1)  = interp1(SOC_1_table, C1_1_table, x_hat_state(1),  "linear","extrap");
    OCV_seg(1) = interp1(SOC_OCV, OCV_charge,    x_hat_state(1),  "linear","extrap");
    
    V_seg(1) = OCV_seg(1) - Vc1_seg(1) - Rs_seg(1)*I(1);
    
    
    
    % (d) Dual EKF Implementation for each sample in this segment
    for k = 2:length(time)
        
        % 1) True SOC update (just for reference/plotting)
        SOC_true(k) = SOC_true(k-1) - (dt / battery_capacity_C) * I(k-1);
        
        % 2) Lookup R1, C1 for the previous step's SOC
        if I(k) < 0  % charging
            R1_seg(k) = interp1(SOC_1_table, R1_1_table, x_hat_state(1), "linear","extrap");
            C1_seg(k) = interp1(SOC_1_table, C1_1_table, x_hat_state(1), "linear","extrap");
        else         % discharging
            R1_seg(k) = interp1(SOC_table, R1_table, x_hat_state(1), "linear","extrap");
            C1_seg(k) = interp1(SOC_table, C1_table, x_hat_state(1), "linear","extrap");
        end
        
        % 3) State EKF - Prediction
        A_state = [1, 0;
                   0, exp(-dt / (R1_seg(k-1)*C1_seg(k-1)))];
        
        x_hat_state_pred(1,1) = SOC_seg(k-1) - (dt / capEst_seg(k-1)) * I(k-1);
        x_hat_state_pred(2,1) = exp(-dt / (R1_seg(k-1)*C1_seg(k-1)))*Vc1_seg(k-1) + ...
                                R1_seg(k-1)*(1 - exp(-dt/(R1_seg(k-1)*C1_seg(k-1)))) * I(k-1);
        
        P_state_pred = A_state * P_state * A_state' + Q_state;
        
        % 4) State EKF - Measurement Update
        if I(k) < 0  % charging
            Rs_seg(k)   = interp1(SOC_1_table, Rs_1_table, x_hat_state_pred(1), "linear","extrap");
            OCV_pred    = interp1(SOC_OCV, OCV_charge, x_hat_state_pred(1), "linear","extrap");
            dOCV_dSOC_k = dOCV_dSOC_1(x_hat_state_pred(1));
            V_pred_state = OCV_pred - x_hat_state_pred(2) - Rs_seg(k)*I(k);
        else
            Rs_seg(k)   = interp1(SOC_table, Rs_table, x_hat_state_pred(1), "linear","extrap");
            OCV_pred    = interp1(SOC_OCV, OCV_discharge, x_hat_state_pred(1), "linear","extrap");
            dOCV_dSOC_k = dOCV_dSOC(x_hat_state_pred(1));
            V_pred_state = OCV_pred - x_hat_state_pred(2) - Rs_seg(k)*I(k);
        end
        
        C_state = [dOCV_dSOC_k, -1];
        
        K_state_previous = K_state;
        K_state = P_state_pred * C_state' / (C_state * P_state_pred * C_state' + R_state);
        
        x_hat_state = x_hat_state_pred + K_state*(V_true(k) - V_pred_state);
        P_state     = P_state_pred - K_state*C_state*P_state_pred;
        
        % Store results
        SOC_seg(k) = x_hat_state(1);
        Vc1_seg(k) = x_hat_state(2);
        V_seg(k)   = OCV_pred - Vc1_seg(k) - Rs_seg(k)*I(k);
        
        % 5) Parameter EKF - Intermediate
        C_param = dOCV_dSOC_k * dt * I(k) / (x_hat_param^2) + [dOCV_dSOC_k, 0]*dx_by_dtheta_k;
        dx_by_dtheta_k = [dt * I(k-1)/(capEst_seg(k-1)^2); 0] + A_state*dx_by_dtheta_k_1;
        dx_by_dtheta_k_1 = dx_by_dtheta_k - K_state_previous*C_param;
        
        % 6) Parameter EKF - Update every "update_interval"
        if mod(k, update_interval) == 0
            P_param_pred = P_param + Q_param;
            K_param = P_param_pred*C_param'/(C_param*P_param_pred*C_param' + R_param);

            x_hat_param = x_hat_param + K_param*(V_true(k) - V_pred_state);
            P_param     = (1 - K_param*C_param)*P_param_pred;

            capEst_seg(k) = x_hat_param;
        else
            % Carry forward
            capEst_seg(k) = capEst_seg(k-1);
        end
        
        % Update beta and propagate second capacity
        currentTime = time(k) + timeOffset;
        if currentTime - last_beta_update_time >= beta_update_interval
            beta_rate = (x_hat_param - last_capacity)/beta_update_interval;
            last_capacity = x_hat_param;
            last_beta_update_time = currentTime;
            %rate = beta / 2000;
            % Store beta and its update time
            allBetaTimes = [allBetaTimes, currentTime];
            allBetaValues = [allBetaValues, beta_rate];
        end
        second_capacity = secondCap_seg(k-1) + beta_rate * dt;
        secondCap_seg(k) = second_capacity;
        
    end
    
    % (e) End of segment: carry over final states to next segment
    C_state = C_state;
    K_state = K_state;
    P_state = P_state;
    C_param = C_param;
    dx_by_dtheta_k = dx_by_dtheta_k;
    dx_by_dtheta_k_1 = dx_by_dtheta_k_1;
    P_param = P_param;
    x_hat_state = x_hat_state;
    x_hat_param = x_hat_param;
    
    % (f) Accumulate segment data into overall arrays for plotting
    segTime = time + timeOffset;  % shift time by offset so each segment is continuous
    
    allTime     = [allTime, segTime];
    allSOC      = [allSOC, SOC_seg];
    allSOC_true = [allSOC_true, SOC_true];
    allV        = [allV, V_seg];
    allV_true   = [allV_true, V_true];
    allCapEst   = [allCapEst, capEst_seg];
    allSecondCap = [allSecondCap, secondCap_seg];
    
    % Record the start/end times for horizontal target lines
    segmentStart(segIdx) = timeOffset;
    segmentEnd(segIdx)   = timeOffset + time(end);
    
    % Advance time offset
    timeOffset = segmentEnd(segIdx);
    capEst_seg_end(segIdx) = capEst_seg(end);
    
end

%% 6) Final Plotting of All Segments
% (a) SOC
% figure;
% plot(allTime, allSOC, 'b', 'LineWidth', 1.5); 
% hold on;
% plot(allTime, allSOC_true, 'r--', 'LineWidth', 1.0);
% xlabel('Time [s]');
% ylabel('SOC');
% legend('Estimated SOC','True SOC (constant capacity)');
% 
% % (b) Voltage
% figure;
% plot(allTime, allV, 'b', 'LineWidth', 1.5); 
% hold on;
% plot(allTime, allV_true, 'r--', 'LineWidth', 1.0);
% xlabel('Time [s]');
% ylabel('Voltage [V]');
% legend('Predicted Voltage','True Voltage');
% 
% % (c) Capacity + Horizontal Lines for Targets + Vertical Lines for Segment Endings
figure;
plot(allTime, allCapEst/3600*1000, 'b', 'LineWidth', 1.5, 'DisplayName','Primary Capacity Estimate');
hold on;
plot(allTime, allSecondCap/3600*1000, 'g', 'LineWidth', 1.5, 'DisplayName','Second Capacity Estimate');
for segIdx = 1:length(cycleSets)
    % Convert the "endCap_Ah" to mAh
    target_mAh = endCap_Ah(segIdx)*1000;
    % Plot a horizontal line from segmentStart to segmentEnd
    if segIdx == 1
        plot([segmentStart(segIdx), segmentEnd(segIdx)], [target_mAh, target_mAh], ...
             'r--', 'LineWidth', 2, 'DisplayName', 'Target Capacity');
    else
        plot([segmentStart(segIdx), segmentEnd(segIdx)], [target_mAh, target_mAh], ...
             'r--', 'LineWidth', 2, 'HandleVisibility', 'off');
    end
    % Plot a vertical line at the end of each segment
    if segIdx == length(cycleSets)
        xline(segmentEnd(segIdx), 'k--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    else
        xline(segmentEnd(segIdx), 'k--', sprintf('End %s', cycleSets{segIdx}), 'LineWidth', 1.5);
    end
end
% Plot the target capacity points at the end of each segment, connected by a line
target_mAh_points = endCap_Ah * 1000;  
plot(segmentEnd, target_mAh_points, 'ro-', 'LineWidth', 2, 'MarkerSize', 8, 'DisplayName','Target Capacity Points');

xlabel('Time [s]');
ylabel('Capacity [mAh]');
legend('Location','best');
title('Capacity Estimates and Target Capacities');
ylim([40,50]);
grid on;

% (d) Beta Evolution Plot
% figure;
% plot(allBetaTimes, allBetaValues/3600*1000, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 6);
% xlabel('Time [s]');
% ylabel('Beta [mAh]');
% title('Evolution of Beta Over Time');
% grid on;
% hold on
% actualBeta = zeros(1, length(cycleSets));
% for i = 1:length(cycleSets)
%     cStart = initCap_Ah(i)*3600;  % coulombs at seg start
%     cEnd   = endCap_Ah(i)*3600;   % coulombs at seg end
%     timeSeg= segmentEnd(i) - segmentStart(i);
% 
%     actualBeta(i) = (cEnd - cStart) / timeSeg;  % coulombs per second
% end
% 
% % Now plot them at segmentEnd, connecting with lines
% plot(segmentEnd, actualBeta, 'ro-','LineWidth',1.8, 'MarkerSize',7,...
%     'DisplayName','Actual \beta');
% legend('Location','best');


% (e) Relative Error of Primary Capacity
% Convert the final estimated capacity from Coulombs to mAh
capEst_seg_end_mAh = capEst_seg_end / 3600 * 1000;

% True target capacities in mAh (endCap_Ah is in Ah)
true_cap_mAh = endCap_Ah * 1000;

% Compute the relative error (%)
relative_error = (capEst_seg_end_mAh - true_cap_mAh) ./ true_cap_mAh * 100;

% Plot the relative error vs. segment end time
figure;
plot(segmentEnd, relative_error, 'bo-', 'LineWidth', 2, 'MarkerSize', 8);
xlabel('Time [s]');
ylabel('Relative Error (%)');
title('Relative Accuracy of Primary Capacity Estimate at Segment Ends');
grid on;
