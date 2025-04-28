%% capacityWeekforecasting.m
%  Dual-EKF with sliding-window β estimation (1e5 s) and 1-week forecast
% -------------------------------------------------------------------------
clear; 

%% 1)  LOOK-UP TABLES & OCV FITS  -----------------------------------------
load('RC_Rs_values_1.mat');
load('RC_Rs_values_1_charging.mat');
load('LIR2032_EEMB_Cell1_25C_OCV.mat');
load('data_Cell2_25C.mat');
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



deg      = 5;
coeff_c  = polyfit(SOC, OCV_charge   , deg);
coeff_d  = polyfit(SOC, OCV_discharge, deg);
dcoeff_c = polyder(coeff_c);
dcoeff_d = polyder(coeff_d);
dOCVdSOC_c = @(s) polyval(dcoeff_c, s);
dOCVdSOC_d = @(s) polyval(dcoeff_d, s);

%% 2)  CYCLE CHUNKS & CAPACITIES  -----------------------------------------
cycleSets  = {'1_20','21_40','41_60','61_80','81_100','101_120',...
              '121_140','141_160','161_180','181_200','201_220',...
              '221_240','241_260'};
initCap_Ah = data_Cell2_25C.plot_capacity(1:end-1)./1000;
endCap_Ah  = data_Cell2_25C.plot_capacity(2:end)  ./1000;
factor     = 1;                                   % no scaling

%% 3)  EKF INITIALISATION  -------------------------------------------------
P_state = diag([1e-7 1e-7]);   
Q_state = diag([1e-6 1]);   
R_state = 0.015;

P_param = 1e-1;                
Q_param = 1e-1;                
R_param = 4.1e-4;

update_interval = 2100;                             % EKF θ-update
beta_update_interval = 1e5;           % not used any more but kept for clarity

x_hat_state = [1; 0];                               % [SOC; Vc1]
x_hat_param = initCap_Ah(1)*3600/factor;            % capacity [C]

%% ►-- 3-A)  β-ESTIMATION & FORECAST SETTINGS  -----------------------------
betaWindow      = 1e5;   % seconds between successive β updates   (≈ 27.8 h)
forecastHorizon = 6e5;   % forecast drawn 6e5 s (≈ 7 d) ahead of anchor

nextBetaTime    = betaWindow;   % t of first β update
anchorTime      = 0;            % start of current β window
anchorCap       = x_hat_param;  % capacity at anchorTime  [C]
beta_curr       = 0;            % current β   [C/s]
forecastEndTime = anchorTime + forecastHorizon;

betaTimes  = [];  betaValues = [];                 % for β-evolution plot

%% 4)  ARRAYS FOR GLOBAL PLOTTING  ----------------------------------------
allTime=[]; allSOC=[]; allSOC_true=[];
allV=[];    allV_true=[];
allCapEst=[];  allCapPred=[];

segmentStart=zeros(1,numel(cycleSets));
segmentEnd  =zeros(1,numel(cycleSets));
timeOffset  =0;

capEst_seg_end = zeros(1,numel(cycleSets));

%% 5)  MAIN 20-CYCLE LOOP  -------------------------------------------------
for seg=1:numel(cycleSets)
    % --- data ------------------------------------------------------------
    dataFile = sprintf('LIR2032_EEMB_Cell2_25C_Aging_Cycles_%s.mat', ...
                       cycleSets{seg});
    S      = load(dataFile);
    dt     = S.dt;
    t_raw  = S.time;
    t_vec  = dt:dt:t_raw(end);

    I      =  interp1(t_raw',-S.I',      t_vec,'linear','extrap');
    V_true =  interp1(t_raw', S.V_true', t_vec,'linear','extrap');
    SOC_0  =  S.SOC_0;
    C_nom  =  initCap_Ah(seg)*3600/factor;           % C

    % --- pre-alloc -------------------------------------------------------
    N   = numel(t_vec);
    SOC = zeros(1,N);  SOC(1)=SOC_0;
    Vc1 = zeros(1,N);  V   = zeros(1,N);
    Rs  = zeros(1,N);  R1  = zeros(1,N);  C1 = zeros(1,N);  OCV=zeros(1,N);
    capEst    = zeros(1,N); capEst(1)=x_hat_param;
    capPred   = NaN(1,N);                           % forecast trace
    SOC_true  = zeros(1,N); SOC_true(1)=SOC_0;

    % --- first-sample look-ups ------------------------------------------
    Rs(1)  = interp1(SOC_1_table,Rs_1_table,SOC_0,'linear','extrap');
    R1(1)  = interp1(SOC_1_table,R1_1_table,SOC_0,'linear','extrap');
    C1(1)  = interp1(SOC_1_table,C1_1_table,SOC_0,'linear','extrap');
    OCV(1) = interp1(SOC_OCV,OCV_charge,SOC_0,'linear','extrap');
    V(1)   = OCV(1) - Rs(1)*I(1);

    K_state = zeros(2,1);
    dx_by_dtheta_k   = [0;0];        
    dx_by_dtheta_k_1 = [0;0];

    % --- SAMPLE LOOP -----------------------------------------------------
    for k=2:N
        % truth SOC
        SOC_true(k)=SOC_true(k-1) - dt/C_nom*I(k-1);

        % lookup R1,C1 table
        if I(k)<0
            R1(k)=interp1(SOC_1_table,R1_1_table,SOC(k-1),'linear','extrap');
            C1(k)=interp1(SOC_1_table,C1_1_table,SOC(k-1),'linear','extrap');
        else
            R1(k)=interp1(SOC_table,R1_table,SOC(k-1),'linear','extrap');
            C1(k)=interp1(SOC_table,C1_table,SOC(k-1),'linear','extrap');
        end

        % prediction
        A=[1 0; 0 exp(-dt/(R1(k-1)*C1(k-1)))];
        x_pred=[SOC(k-1)-dt/capEst(k-1)*I(k-1);
                exp(-dt/(R1(k-1)*C1(k-1)))*Vc1(k-1)+ ...
                R1(k-1)*(1-exp(-dt/(R1(k-1)*C1(k-1))))*I(k-1)];
        P_pred=A*P_state*A'+Q_state;

        % measurement
        if I(k)<0
            Rs(k)=interp1(SOC_1_table,Rs_1_table,x_pred(1),'linear','extrap');
            OCVpred=interp1(SOC_OCV,OCV_charge,x_pred(1),'linear','extrap');
            dOCV   = dOCVdSOC_c(x_pred(1));
        else
            Rs(k)=interp1(SOC_table,Rs_table,x_pred(1),'linear','extrap');
            OCVpred=interp1(SOC_OCV,OCV_discharge,x_pred(1),'linear','extrap');
            dOCV   = dOCVdSOC_d(x_pred(1));
        end
        V_pred = OCVpred - x_pred(2) - Rs(k)*I(k);
        Cst=[dOCV -1];
        K_state=P_pred*Cst'/(Cst*P_pred*Cst'+R_state);

        x_hat=x_pred + K_state*(V_true(k)-V_pred);
        P_state=P_pred - K_state*Cst*P_pred;

        SOC(k)=x_hat(1); Vc1(k)=x_hat(2);
        V(k)=OCVpred - Vc1(k) - Rs(k)*I(k);

        % parameter EKF

        C_param = dOCV*dt*I(k)/(x_hat_param^2) + [dOCV 0]*dx_by_dtheta_k;
        dx_by_dtheta_k = [dt*I(k-1)/(capEst(k-1)^2); 0] ...
                         + A*dx_by_dtheta_k_1;
        dx_by_dtheta_k_1 = dx_by_dtheta_k - K_state*C_param;

        if mod(k,update_interval)==0
            P_param_pred = P_param + Q_param;
            K_param = P_param_pred*C_param'/(C_param*P_param_pred*C_param' + R_param);
            x_hat_param = x_hat_param + K_param*(V_true(k) - V_pred);
            P_param     = (1 - K_param*C_param)*P_param_pred;
        end
        capEst(k) = x_hat_param;

        %% ►-- β-WINDOW & FORECAST ---------------------------------------
        tG = t_vec(k)+timeOffset;           % global time (s)

        % close β window?
        if tG>=nextBetaTime
            beta_curr  = (capEst(k)-anchorCap)/(tG-anchorTime);
            betaTimes  = [betaTimes, nextBetaTime];
            betaValues = [betaValues, beta_curr];

            anchorTime = tG;                          % reset anchor
            anchorCap  = capEst(k);
            nextBetaTime   = nextBetaTime + betaWindow;
            forecastEndTime= anchorTime + forecastHorizon;
        end

        % draw forecast within current horizon
        if tG>=anchorTime && tG<=forecastEndTime
            capPred(k)=anchorCap + beta_curr * forecastHorizon;
        end
        % ---------------------------------------------------------------
    end % sample loop

    % --- carry-over & accumulate ----------------------------------------
    segT = t_vec+timeOffset;
    allTime=[allTime segT];
    allSOC=[allSOC SOC];          allSOC_true=[allSOC_true SOC_true];
    allV  =[allV   V];            allV_true  =[allV_true   V_true];
    allCapEst=[allCapEst capEst];
    allCapPred=[allCapPred capPred];

    segmentStart(seg)=timeOffset;
    segmentEnd(seg)=timeOffset+t_vec(end);
    timeOffset=segmentEnd(seg);
    capEst_seg_end(seg)=capEst(end);
end
% -------------------------------------------------------------------------

%% 6)  PLOTTING  -----------------------------------------------------------
figure;
plot(allTime,allCapEst/3600*1000,'b','LineWidth',1.4,...
     'DisplayName','EKF Capacity');
hold on;
plot(allTime+6e5,allCapPred/3600*1000,'m--','LineWidth',1.7,...
     'DisplayName','β-Forecast (6 × 10⁵ s)');
for s=1:numel(cycleSets)
    yT=endCap_Ah(s)*1000;
    if s==1
        plot([segmentStart(s) segmentEnd(s)],[yT yT],'r-.','LineWidth',2,...
             'DisplayName','Target Capacity');
    else
        plot([segmentStart(s) segmentEnd(s)],[yT yT],'r-.','LineWidth',2,...
             'HandleVisibility','off');
    end
    xline(segmentEnd(s),'k--','LineWidth',1.1,'HandleVisibility','off');
end
plot(segmentEnd,endCap_Ah*1000,'ro-','LineWidth',2,'MarkerSize',7,...
     'DisplayName','Target Points');
xlabel('Time [s]'); ylabel('Capacity [mAh]');
title('Capacity EKF & One-Week Forecast (β refresh every 1 × 10⁵ s)');
legend('Location','best'); grid on; ylim([40 50]);

% β evolution (optional)
figure;
stairs(betaTimes,betaValues/3600*1000,'b-o','LineWidth',1.4);
xlabel('Time [s]'); ylabel('\beta [mAh s^{-1}]');
title('Sliding-Window β (Δt = 1 × 10⁵ s)'); grid on;

% relative error at segment ends
capEst_mAh=capEst_seg_end/3600*1000; true_mAh=endCap_Ah*1000;
relErr=(capEst_mAh-true_mAh)./true_mAh*100;
figure;
plot(segmentEnd,relErr,'bo-','LineWidth',2,'MarkerSize',7);
xlabel('Time [s]'); ylabel('Relative Error [%]');
title('Primary-Capacity Relative Error @ Segment End'); grid on;
