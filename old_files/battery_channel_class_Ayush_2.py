import numpy as np
from scipy.interpolate import interp1d
from scipy.io import loadmat # Load the MATLAB data files
class battery_channel:
    def __init__(self, channel, state, mode, cycle_count, volt_v, temp_c, chg_val, dis_val):
        self.channel = channel
        self.state = state
        self.state_prev = state
        self.mode = mode
        self.test_sequence = 0
        self.volt_v = volt_v
        self.temp_c = temp_c
        self.curr_ma = 0
        self.cycle_count = cycle_count
        self.time_resting_started_s = -1
        self.time_prev_s = -1
        self.chg_val = chg_val
        self.dis_val = dis_val
        self.chg_low_val = 200
        self.dis_low_val = dis_val
        self.pulse_state = False
        self.en_chg_state = False
        self.en_dis_state = False
        self.en_cur_state = False
        self.update_act = False
        self.cc_capacity_mas = 0
        self.cc_soc_mas = 0
        self.est_capacity_mas = 0.0494  # Initial capacity in Ah
        self.est_soc = 0
        self.est_volt_v = 0
        self.est_cov = np.zeros((2, 2))  # 2x2 covariance matrix for state EKF

        # lookup tables as class attributes
        rc_rs_values = loadmat(r'C:\Users\ayushp5\OneDrive - University of California, Davis\Lin Lab\Proteus Space - AF project\battery test files\EEMB battery files\python\RC_Rs_values_1.mat') 
        rc_rs_values_charging = loadmat(r'C:\Users\ayushp5\OneDrive - University of California, Davis\Lin Lab\Proteus Space - AF project\battery test files\EEMB battery files\python\RC_Rs_values_1_charging.mat')
        ocv_data = loadmat(r'C:\Users\ayushp5\OneDrive - University of California, Davis\Lin Lab\Proteus Space - AF project\battery test files\EEMB battery files\python\LIR2032_EEMB_Cell1_25C_OCV.mat')
        data_cell2 = loadmat(r'C:\Users\ayushp5\OneDrive - University of California, Davis\Lin Lab\Proteus Space - AF project\battery test files\EEMB battery files\python\data_Cell2_25C.mat')
    
        
        # self.SOC_table = np.linspace(0, 1, 100)  # Discharging SOC
        # self.Rs_table = np.ones(100) * 0.1       # Discharging Rs
        # self.R1_table = np.ones(100) * 0.2       # Discharging R1
        # self.C1_table = np.ones(100) * 1000      # Discharging C1
        # self.SOC_1_table = np.linspace(0, 1, 100) # Charging SOC
        # self.Rs_1_table = np.ones(100) * 0.1      # Charging Rs
        # self.R1_1_table = np.ones(100) * 0.2      # Charging R1
        # self.C1_1_table = np.ones(100) * 1000     # Charging C1
        # self.SOC_OCV = np.linspace(0, 1, 100)     # OCV SOC
        # self.OCV_charge = np.linspace(3.0, 4.2, 100)  # OCV charging
        # self.OCV_discharge = np.linspace(3.0, 4.2, 100)  # OCV discharging


        # Extract lookup table values - Discharging params
        self.lookup_table = rc_rs_values['lookupTable']
        self.SOC_table = lookup_table['SOC'][0][0].flatten()
        self.Rs_table = lookup_table['Rs'][0][0].flatten()
        self.R1_table = lookup_table['R1'][0][0].flatten()
        self.C1_table = lookup_table['C1'][0][0].flatten()

        # Extract lookup table values - Charging params
        self.lookup_table_1 = rc_rs_values_charging['lookupTable_1'][0][0]
        self.SOC_1_table = lookup_table_1['SOC'][0][0].flatten()
        self.Rs_1_table = lookup_table_1['Rs'][0][0].flatten()
        self.R1_1_table = lookup_table_1['R1'][0][0].flatten()
        self.C1_1_table = lookup_table_1['C1'][0][0].flatten()

        # OCV data
        self.SOC_OCV = ocv_data['SOC'].flatten()
        self.OCV_charge = ocv_data['OCV_charge'].flatten()
        self.OCV_discharge = ocv_data['OCV_discharge'].flatten()

        # Polynomial fit for OCV 
        degree = 5
        self.coefficients_1 = np.polyfit(self.SOC_OCV, self.OCV_charge, degree)
        self.coefficients = np.polyfit(self.SOC_OCV, self.OCV_discharge, degree)
        self.derivative_coefficients_1 = np.polyder(self.coefficients_1)
        self.derivative_coefficients = np.polyder(self.coefficients)

        # EKF initializations (same as reference code)
        self.P_state = np.diag([1e-7, 1e-7])     # State covariance
        self.Q_state = np.diag([1e-6, 1e-3])     # Process noise covariance
        self.R_state = 0.015                     # Measurement noise covariance
        self.P_param = 1e-1                      # Parameter covariance
        self.Q_param = 1e-1                      # Parameter process noise
        self.R_param = 3.72725e-3                # Parameter measurement noise
        self.x_hat_state = np.array([0.0, 0.0])  # [SOC; Vc1]
        self.x_hat_param = 0.0464                  # Initial capacity (Ah)
        self.K_state = np.zeros(2)
        self.dx_by_dtheta_k = np.zeros(2)
        self.dx_by_dtheta_k_1 = np.zeros(2)
        self.update_counter = 0

    def dOCV_dSOC_1(self, soc):
        return np.polyval(self.derivative_coefficients_1, soc)

    def dOCV_dSOC(self, soc):
        return np.polyval(self.derivative_coefficients, soc)

    def channel_logic(self, time_iter_s, PARAMS):
        
        if self.cycle_count < PARAMS.NUM_CYCLES_PER_TEST:
            self.mode = 'CYCLE'
            if self.state == 'CHG' and self.volt_v >= PARAMS.CHG_LIMIT_V:
                self.state = 'CHG_REST'
                self.time_resting_started_s = time_iter_s
            if self.state == 'DIS' and self.volt_v <= PARAMS.DIS_LIMIT_V:
                self.state = 'DIS_REST'
                self.time_resting_started_s = time_iter_s
            if self.state == 'CHG_REST' and time_iter_s - self.time_resting_started_s > PARAMS.TIME_CYCLE_REST_S:
                self.state = 'DIS'
            if self.state == 'DIS_REST' and time_iter_s - self.time_resting_started_s > PARAMS.TIME_CYCLE_REST_S:
                self.state = 'CHG'
                self.cycle_count += 1
        else:
            self.mode = 'TEST'
            if self.test_sequence == 0 and self.volt_v >= PARAMS.DIS_LIMIT_V:
                self.state = 'DIS_LOW'
            if self.test_sequence == 0 and self.volt_v < PARAMS.DIS_LIMIT_V:
                self.state = 'REST'
                self.test_sequence = 1
                self.time_resting_started_s = time_iter_s
                self.cc_soc_mas = 0
            if self.test_sequence == 1 and time_iter_s - self.time_resting_started_s > 2*PARAMS.TIME_TEST_REST_S:
                self.state = 'CHG_LOW'
                self.test_sequence = 2
                self.time_prev_s = time_iter_s
            if self.test_sequence == 2 and self.volt_v <= PARAMS.CHG_LIMIT_V:
                self.cc_soc_mas = self.cc_soc_mas - self.curr_ma * (time_iter_s - self.time_prev_s)
                self.time_prev_s = time_iter_s
            if self.test_sequence == 2 and self.volt_v > PARAMS.CHG_LIMIT_V:
                self.state = 'REST'
                self.test_sequence = 3
                self.cc_capacity_mas = self.cc_soc_mas
                self.time_resting_started_s = time_iter_s
            if self.test_sequence == 3 and time_iter_s - self.time_resting_started_s > 2*PARAMS.TIME_TEST_REST_S:
                self.state = 'DIS'
                self.pulse_state = True
                self.test_sequence = 4
                self.time_resting_started_s = time_iter_s
            if self.test_sequence == 4 and time_iter_s - self.time_resting_started_s > PARAMS.TIME_PULSE_TEST_S:
                self.state = 'REST'
                self.pulse_state = False
                self.test_sequence = 5
                self.time_resting_started_s = time_iter_s
            if self.test_sequence == 5 and time_iter_s - self.time_resting_started_s > PARAMS.TIME_PULSE_REST_S:
                self.state = 'CHG'
                self.pulse_state = True
                self.test_sequence = 6
                self.time_resting_started_s = time_iter_s
            if self.test_sequence == 6 and time_iter_s - self.time_resting_started_s > PARAMS.TIME_PULSE_TEST_S:
                self.state = 'REST'
                self.pulse_state = False
                self.test_sequence = 7
                self.time_resting_started_s = time_iter_s
            if self.test_sequence == 7 and time_iter_s - self.time_resting_started_s > PARAMS.TIME_PULSE_REST_S:
                self.state = 'DIS'
                self.test_sequence = 8
                self.time_prev_s = time_iter_s
            if self.test_sequence == 8 and self.cc_soc_mas >= PARAMS.SOC_PULSE1 * self.cc_capacity_mas and self.volt_v >= PARAMS.FB_PULSE1_V:
                self.cc_soc_mas = self.cc_soc_mas - self.curr_ma * (time_iter_s - self.time_prev_s)
                self.time_prev_s = time_iter_s
            if self.test_sequence == 8 and (self.cc_soc_mas < PARAMS.SOC_PULSE1 * self.cc_capacity_mas or self.volt_v < PARAMS.FB_PULSE1_V):
                self.state = 'REST'
                self.test_sequence = 9
                self.time_resting_started_s = time_iter_s
            if self.test_sequence == 9 and time_iter_s - self.time_resting_started_s > PARAMS.TIME_TEST_REST_S:
                self.state = 'DIS'
                self.pulse_state = True
                self.test_sequence = 10
                self.time_resting_started_s = time_iter_s
            if self.test_sequence == 10 and time_iter_s - self.time_resting_started_s > PARAMS.TIME_PULSE_TEST_S:
                self.state = 'REST'
                self.pulse_state = False
                self.test_sequence = 11
                self.time_resting_started_s = time_iter_s
            if self.test_sequence == 11 and time_iter_s - self.time_resting_started_s > PARAMS.TIME_PULSE_REST_S:
                self.state = 'CHG'
                self.pulse_state = True
                self.test_sequence = 12
                self.time_resting_started_s = time_iter_s
            if self.test_sequence == 12 and time_iter_s - self.time_resting_started_s > PARAMS.TIME_PULSE_TEST_S:
                self.state = 'REST'
                self.pulse_state = False
                self.test_sequence = 13
                self.time_resting_started_s = time_iter_s
            if self.test_sequence == 13 and time_iter_s - self.time_resting_started_s > PARAMS.TIME_PULSE_REST_S:
                self.state = 'DIS'
                self.test_sequence = 14
                self.time_prev_s = time_iter_s
            if self.test_sequence == 14 and self.cc_soc_mas >= PARAMS.SOC_PULSE2 * self.cc_capacity_mas and self.volt_v >= PARAMS.FB_PULSE2_V:
                self.cc_soc_mas = self.cc_soc_mas - self.curr_ma * (time_iter_s - self.time_prev_s)
                self.time_prev_s = time_iter_s
            if self.test_sequence == 14 and (self.cc_soc_mas < PARAMS.SOC_PULSE2 * self.cc_capacity_mas or self.volt_v < PARAMS.FB_PULSE2_V):
                self.state = 'REST'
                self.test_sequence = 15
                self.time_resting_started_s = time_iter_s
            if self.test_sequence == 15 and time_iter_s - self.time_resting_started_s > PARAMS.TIME_TEST_REST_S:
                self.state = 'DIS'
                self.pulse_state = True
                self.test_sequence = 16
                self.time_resting_started_s = time_iter_s
            if self.test_sequence == 16 and time_iter_s - self.time_resting_started_s > PARAMS.TIME_PULSE_TEST_S:
                self.state = 'REST'
                self.pulse_state = False
                self.test_sequence = 17
                self.time_resting_started_s = time_iter_s
            if self.test_sequence == 17 and time_iter_s - self.time_resting_started_s > PARAMS.TIME_PULSE_REST_S:
                self.state = 'CHG'
                self.pulse_state = True
                self.test_sequence = 18
                self.time_resting_started_s = time_iter_s
            if self.test_sequence == 18 and time_iter_s - self.time_resting_started_s > PARAMS.TIME_PULSE_TEST_S:
                self.state = 'REST'
                self.pulse_state = False
                self.test_sequence = 19
                self.time_resting_started_s = time_iter_s
            if self.test_sequence == 19 and time_iter_s - self.time_resting_started_s > PARAMS.TIME_PULSE_REST_S:
                self.state = 'DIS'
                self.test_sequence = 20
            if self.test_sequence == 20 and self.volt_v < PARAMS.DIS_LIMIT_V:
                self.state = 'REST'
                self.test_sequence = 21
                self.time_resting_started_s = time_iter_s
            if self.test_sequence == 21 and time_iter_s - self.time_resting_started_s > 2*PARAMS.TIME_TEST_REST_S:
                self.mode = 'CYCLE'
                self.test_sequence = 0
                self.state = 'CHG'
                self.cycle_count = 0

    def channel_action(self, PARAMS):
        charge_setpoint_ma = 0
        discharge_setpoint_ma = 0
        self.update_act = True

        if self.state == 'CHG':
            charge_setpoint_ma = 0
            self.update_act = False
            if self.temp_c > PARAMS.TEMP_CHG_MIN_C:
                charge_setpoint_ma = PARAMS.CHG_MIN_SETPT_MA
            if self.temp_c > PARAMS.TEMP_CHG_LOWER_C:
                charge_setpoint_ma = PARAMS.CHG_LOWER_SETPT_MA
            if self.temp_c > PARAMS.TEMP_CHG_UPPER_C:
                charge_setpoint_ma = PARAMS.CHG_UPPER_SETPT_MA
            if self.temp_c > PARAMS.TEMP_CHG_MAX_C:
                charge_setpoint_ma = 0

            if charge_setpoint_ma != 0:
                if self.curr_ma < charge_setpoint_ma - PARAMS.CHG_SETPT_DELTA_MA:
                    self.chg_val += 0.1
                    self.chg_val = min(260, self.chg_val)
                    self.update_act = True
                elif self.curr_ma > charge_setpoint_ma + PARAMS.CHG_SETPT_DELTA_MA:
                    self.chg_val -= 0.1
                    self.chg_val = max(0, self.chg_val)
                    self.update_act = True

        if self.state == 'CHG_LOW':
            charge_setpoint_ma = 0
            self.update_act = False
            if self.temp_c > PARAMS.TEMP_CHG_MIN_C and self.temp_c < PARAMS.TEMP_CHG_MAX_C:
                charge_setpoint_ma = PARAMS.CHG_LOW_SETPT_MA
                if self.curr_ma < charge_setpoint_ma - PARAMS.CHG_LOW_SETPT_DELTA_MA:
                    self.chg_low_val += 0.1
                    self.chg_low_val = min(260, self.chg_low_val)
                    self.update_act = True
                elif self.curr_ma > charge_setpoint_ma + PARAMS.CHG_LOW_SETPT_DELTA_MA:
                    self.chg_low_val -= 0.1
                    self.chg_low_val = max(0, self.chg_low_val)
                    self.update_act = True

        if self.state == 'DIS':
            discharge_setpoint_ma = 0
            self.update_act = False
            if self.temp_c > PARAMS.TEMP_MIN_C and self.temp_c < PARAMS.TEMP_MAX_C:
                discharge_setpoint_ma = PARAMS.DIS_SETPT_MA
                if self.curr_ma < discharge_setpoint_ma - PARAMS.DIS_SETPT_DELTA_MA:
                    self.dis_val += 0.1
                    self.dis_val = min(170, self.dis_val)
                    self.update_act = True
                elif self.curr_ma > discharge_setpoint_ma + PARAMS.DIS_SETPT_DELTA_MA:
                    self.dis_val -= 0.1
                    self.dis_val = max(0, self.dis_val)
                    self.update_act = True

        if self.state == 'DIS_LOW':
            discharge_setpoint_ma = 0
            self.update_act = False
            if self.temp_c > PARAMS.TEMP_MIN_C and self.temp_c < PARAMS.TEMP_MAX_C:
                discharge_setpoint_ma = PARAMS.DIS_LOW_SETPT_MA
                if self.curr_ma < discharge_setpoint_ma - PARAMS.DIS_LOW_SETPT_DELTA_MA:
                    self.dis_low_val += 0.1
                    self.dis_low_val = min(170, self.dis_low_val)
                    self.update_act = True
                elif self.curr_ma > discharge_setpoint_ma + PARAMS.DIS_LOW_SETPT_DELTA_MA:
                    self.dis_low_val -= 0.1
                    self.dis_low_val = max(0, self.dis_low_val)
                    self.update_act = True

        self.en_cur_state = False
        self.en_dis_state = False
        self.en_chg_state = False
        if discharge_setpoint_ma > 0:
            self.en_dis_state = True
            if discharge_setpoint_ma > PARAMS.DIS_TRANS_MA:
                self.en_cur_state = True
        if charge_setpoint_ma < 0:
            self.en_chg_state = True

    def state_estimate(self, meas_volt_v, meas_curr_ma, time_iter_s, PARAMS):
        # Initialize capacity if not set
        if self.x_hat_param == 0.0:
            self.x_hat_param = PARAMS.BATTERY_CAPACITY_AH * 3600  # Convert Ah to As

        # Compute time step
        dt = time_iter_s - self.time_prev_s if self.time_prev_s != -1 else 0.1
        if dt <= 0:
            dt = 0.1  # Avoid division by zero

        # Update current
        self.curr_ma = meas_curr_ma

        # Lookup R1, C1 based on charging/discharging
        if meas_curr_ma < 0:  # Charging
            R1 = interp1d(self.SOC_1_table, self.R1_1_table, kind='linear', fill_value='extrapolate')(self.x_hat_state[0])
            C1 = interp1d(self.SOC_1_table, self.C1_1_table, kind='linear', fill_value='extrapolate')(self.x_hat_state[0])
        else:  # Discharging
            R1 = interp1d(self.SOC_table, self.R1_table, kind='linear', fill_value='extrapolate')(self.x_hat_state[0])
            C1 = interp1d(self.SOC_table, self.C1_table, kind='linear', fill_value='extrapolate')(self.x_hat_state[0])

        # State EKF - Prediction
        A_state = np.array([[1, 0],
                            [0, np.exp(-dt / (R1 * C1))]])
        x_hat_state_pred = np.zeros(2)
        x_hat_state_pred[0] = self.x_hat_state[0] - (dt / self.x_hat_param) * meas_curr_ma
        x_hat_state_pred[1] = (np.exp(-dt / (R1 * C1)) * self.x_hat_state[1] +
                              R1 * (1 - np.exp(-dt / (R1 * C1))) * meas_curr_ma)
        P_state_pred = A_state @ self.P_state @ A_state.T + self.Q_state

        # State EKF - Measurement Update
        if meas_curr_ma < 0:  # Charging
            Rs = interp1d(self.SOC_1_table, self.Rs_1_table, kind='linear', fill_value='extrapolate')(x_hat_state_pred[0])
            OCV_pred = interp1d(self.SOC_OCV, self.OCV_charge, kind='linear', fill_value='extrapolate')(x_hat_state_pred[0])
            dOCV_dSOC_k = self.dOCV_dSOC_1(x_hat_state_pred[0])
            V_pred_state = OCV_pred - x_hat_state_pred[1] - Rs * meas_curr_ma
        else:  # Discharging
            Rs = interp1d(self.SOC_table, self.Rs_table, kind='linear', fill_value='extrapolate')(x_hat_state_pred[0])
            OCV_pred = interp1d(self.SOC_OCV, self.OCV_discharge, kind='linear', fill_value='extrapolate')(x_hat_state_pred[0])
            dOCV_dSOC_k = self.dOCV_dSOC(x_hat_state_pred[0])
            V_pred_state = OCV_pred - x_hat_state_pred[1] - Rs * meas_curr_ma

        C_state = np.array([dOCV_dSOC_k, -1])
        K_state_previous = self.K_state.copy()
        denom = C_state @ P_state_pred @ C_state.T + self.R_state
        if abs(denom) < 1e-10:
            denom = 1e-10
        self.K_state = P_state_pred @ C_state / denom
        self.x_hat_state = x_hat_state_pred + self.K_state * (meas_volt_v - V_pred_state)
        self.P_state = P_state_pred - np.outer(self.K_state, C_state) @ P_state_pred

        # Parameter EKF - Intermediate
        C_param = dOCV_dSOC_k * dt * meas_curr_ma / (self.x_hat_param**2) + np.array([dOCV_dSOC_k, 0]) @ self.dx_by_dtheta_k
        self.dx_by_dtheta_k = np.array([dt * meas_curr_ma / (self.x_hat_param**2), 0]) + A_state @ self.dx_by_dtheta_k_1
        self.dx_by_dtheta_k_1 = self.dx_by_dtheta_k - K_state_previous * C_param

        # Parameter EKF - Update
        self.update_counter += 1
        if self.update_counter % 1000 == 0:
            P_param_pred = self.P_param + self.Q_param
            denom_param = C_param * P_param_pred * C_param + self.R_param
            if abs(denom_param) < 1e-10:
                denom_param = 1e-10
            K_param = P_param_pred * C_param / denom_param
            self.x_hat_param = self.x_hat_param + K_param * (meas_volt_v - V_pred_state)
            self.P_param = (1 - K_param * C_param) * P_param_pred
            self.update_counter = 0

        # Update outputs (states)
        self.est_soc = self.x_hat_state[0]
        self.est_volt_v = self.x_hat_state[1]
        self.cc_capacity_est = self.x_hat_param
        # Update outputs (parameters)
        self.est_cov = self.P_state
        self.K_state
        self.x_hat_state
        self.dx_by_dtheta_k
        


