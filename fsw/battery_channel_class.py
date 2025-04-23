import numpy as np
import json
from scipy.interpolate import interp1d
from scipy.io import loadmat # Load the MATLAB data files
class battery_channel:
    def __init__(self, channel, state, mode, cycle_count, volt_v, temp_c, chg_val, dis_val, file_name, ekf_cap_file, cyc_cap_file):
        self.file_name = file_name
        self.channel = channel
        self.curr_ma = 0
        self.meas_curr_a_k_1 = 0
        self.time_resting_started_s = -1
        self.time_prev_s = -1
        self.time_iter_prev_s = -1
        self.update_act = False
        self.R_state = 0.015                     # Measurement noise covariance
        self.Q_state = np.diag([1e-6, 1e-3])     # Process noise covariance
        self.Q_param = 1e-1                      # Parameter process noise
        self.R_param = 3.72725e-3                # Parameter measurement noise
        self.update_counter = 0
        self.est_volt_v = 0
        self.pred_cyc_one = 0
        self.pred_cyc_two = 0
        self.pred_ekf_one = 0
        self.pred_ekf_two = 0
        
        # load cap estimate backups
        try:
            temp = np.load(ekf_cap_file)
            self.ekf_cap_est_mah = temp[:,channel]
            temp = np.load(cyc_cap_file)
            self.cyc_cap_est_mah = temp[:,channel]
        except Exception:
            print('Error reading capacity files')
            self.ekf_cap_est_mah = -1*np.ones([1200])
            self.cyc_cap_est_mah = -1*np.ones([ 360])
        
        try:
            with open(file_name, 'r') as json_in:
                vals = json.loads(json_in.read())
                self.state      = vals["state"]             
                self.state_prev = vals["state_prev"]           
                self.mode = vals["mode"]
                self.test_sequence = vals["test_sequence"]
                self.volt_v = vals["volt_v"]
                self.temp_c = vals["temp_c"]
                self.cycle_count = vals["cycle_count"]
                self.chg_val = vals["chg_val"]
                self.dis_val = vals["dis_val"]
                self.chg_low_val = vals["chg_low_val"]
                self.dis_low_val = vals["dis_low_val"]
                self.pulse_state = vals["pulse_state"]
                self.en_chg_state = vals["en_chg_state"]
                self.en_dis_state = vals["en_dis_state"]
                self.en_cur_state = vals["en_cur_state"]
                self.cc_capacity_mas = vals["cc_capacity_mas"]
                self.cc_soc_mas = vals["cc_soc_mas"]
                self.R_SHUNT_OHMS = vals["R_SHUNT_OHMS"]
                
                self.est_capacity_as = vals["est_capacity_as"]
                self.est_soc = vals["est_soc"]
                self.est_cov_state = np.zeros((2, 2)) # 2x2 covariance matrix for state EKF
                self.est_cov_state[0,0] = vals["est_cov_state00"]
                self.est_cov_state[0,1] = vals["est_cov_state01"] 
                self.est_cov_state[1,0] = vals["est_cov_state10"] 
                self.est_cov_state[1,1] = vals["est_cov_state11"] 
                self.est_cov_param = vals["est_cov_param"] #covariance for param EKF
                
                # EKF initializations 
                self.P_state = self.est_cov_state     # State covariance
                self.P_param = self.est_cov_param     # Parameter covariance
                
        except Exception:
            print('Error reading battery json')
            self.state = state
            self.state_prev = state
            self.mode = mode
            self.test_sequence = 0
            self.volt_v = volt_v     #most recent voltage measurement
            self.temp_c = temp_c
            self.curr_ma = 0
            self.meas_curr_a_k_1 = 0
            self.cycle_count = cycle_count
            self.time_resting_started_s = -1
            self.time_prev_s = -1
            self.time_iter_prev_s = -1
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
            self.R_SHUNT_OHMS = 1.0
            
            self.est_capacity_as = 0.0476705 * 3600   # Initial capacity in As
            self.est_soc = 0
            self.est_volt_v = 0
            self.est_cov_state = np.zeros((2, 2))  # 2x2 covariance matrix for state EKF
            self.est_cov_param = -1 #covariance for param EKF
                
            # EKF initializations 
            self.P_state = np.diag([1e-7, 1e-7])     # State covariance
            self.P_param = 1e-1                      # Parameter covariance
        
        self.x_hat_state = np.array([self.est_soc, self.est_volt_v])  # [SOC; Vc1]
        self.x_hat_param = self.est_capacity_as                # Initial capacity (As)
        self.x_hat_param_k_1 = self.est_capacity_as
        
        self.K_state = np.zeros(2)
        self.dx_by_dtheta_k = np.zeros(2)
        self.dx_by_dtheta_k_1 = np.zeros(2)

        # lookup tables as class attributes
        rc_rs_values = loadmat(r'mat_files/RC_Rs_values_1.mat') 
        rc_rs_values_charging = loadmat(r'mat_files/RC_Rs_values_1_charging.mat')
        ocv_data = loadmat(r'mat_files/LIR2032_EEMB_Cell1_25C_OCV.mat')
        data_cell2 = loadmat(r'mat_files/data_Cell2_25C.mat')

        # Extract lookup table values - Discharging params
        self.lookup_table = rc_rs_values['lookupTable']
        self.SOC_table = self.lookup_table['SOC'][0][0].flatten()
        self.Rs_table = self.lookup_table['Rs'][0][0].flatten()
        self.R1_table = self.lookup_table['R1'][0][0].flatten()
        self.C1_table = self.lookup_table['C1'][0][0].flatten()

        # Extract lookup table values - Charging params
        self.lookup_table_1 = rc_rs_values_charging['lookupTable_1'][0][0]
        self.SOC_1_table = self.lookup_table_1['SOC'][0][0].flatten()
        self.Rs_1_table = self.lookup_table_1['Rs'][0][0].flatten()
        self.R1_1_table = self.lookup_table_1['R1'][0][0].flatten()
        self.C1_1_table = self.lookup_table_1['C1'][0][0].flatten()

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
                # move all entries down to make room for new entry
                temp = self.cyc_cap_est_mah[0] #latest entry
                if temp < 0:
                    temp = (1/3600)*self.cc_soc_mas
                self.cyc_cap_est_mah[1:]=self.cyc_cap_est_mah[:-1]
                self.cyc_cap_est_mah[0] = (1-PARAMS.ALPHA_CYC)*temp + PARAMS.ALPHA_CYC*(1/3600)*self.cc_soc_mas
            elif self.state == 'CHG':
                self.cc_soc_mas = self.cc_soc_mas - self.curr_ma * (time_iter_s - self.time_prev_s)
                self.time_prev_s = time_iter_s
            
            if self.state == 'DIS' and self.volt_v <= PARAMS.DIS_LIMIT_V:
                self.state = 'DIS_REST'
                self.time_resting_started_s = time_iter_s
                # move all entries down to make room for new entry
                temp = self.cyc_cap_est_mah[0] #latest entry
                if temp < 0:
                    temp = (1/3600)*self.cc_soc_mas
                self.cyc_cap_est_mah[1:]=self.cyc_cap_est_mah[:-1]
                self.cyc_cap_est_mah[0] = (1-PARAMS.ALPHA_CYC)*temp + PARAMS.ALPHA_CYC*(1/3600)*self.cc_soc_mas
            elif self.state == 'DIS':
                self.cc_soc_mas = self.cc_soc_mas - self.curr_ma * (time_iter_s - self.time_prev_s)
                self.time_prev_s = time_iter_s
            
            if self.state == 'CHG_REST' and time_iter_s - self.time_resting_started_s > PARAMS.TIME_CYCLE_REST_S:
                self.state = 'DIS'
                self.cc_soc_mas = 0
                self.time_prev_s = time_iter_s
            if self.state == 'DIS_REST' and time_iter_s - self.time_resting_started_s > PARAMS.TIME_CYCLE_REST_S:
                self.state = 'CHG'
                self.cycle_count += 1
                self.cc_soc_mas = 0
                self.time_prev_s = time_iter_s
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
                if self.curr_ma < charge_setpoint_ma - 3*PARAMS.CHG_SETPT_DELTA_MA:
                    self.chg_val += 0.8
                    self.chg_val = min(260, self.chg_val)
                    self.update_act = True
                elif self.curr_ma < charge_setpoint_ma - PARAMS.CHG_SETPT_DELTA_MA:
                    self.chg_val += 0.1
                    self.chg_val = min(260, self.chg_val)
                    self.update_act = True
                elif self.curr_ma > charge_setpoint_ma + 3*PARAMS.CHG_SETPT_DELTA_MA:
                    self.chg_val -= 0.8
                    self.chg_val = max(0, self.chg_val)
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

    def state_estimate(self, meas_volt_v, meas_curr_ma, time_iter_s):
        meas_curr_a = meas_curr_ma / 1000
         
        # Compute time step
        dt = time_iter_s - self.time_iter_prev_s
        if self.time_iter_prev_s < 0:
            dt = 1
        dt = max(0.01, min(10, dt)) # avoid cases with large or small dt
        self.time_iter_prev_s = time_iter_s

        # Update current
        #self.curr_ma = meas_curr_ma

        # Lookup R1, C1 based on charging/discharging
        if self.meas_curr_a_k_1 < 0:  # Charging
            #R1 = interp1d(self.SOC_1_table, self.R1_1_table, kind='linear', fill_value='extrapolate')(self.x_hat_state[0])
            R1 = np.interp(self.x_hat_state[0],self.SOC_1_table, self.R1_1_table);
            #C1 = interp1d(self.SOC_1_table, self.C1_1_table, kind='linear', fill_value='extrapolate')(self.x_hat_state[0])
            C1 = np.interp(self.x_hat_state[0],self.SOC_1_table, self.C1_1_table);
            
        else:  # Discharging
            #R1 = interp1d(self.SOC_table, self.R1_table, kind='linear', fill_value='extrapolate')(self.x_hat_state[0])
            R1 = np.interp(self.x_hat_state[0],self.SOC_table, self.R1_table);
            #C1 = interp1d(self.SOC_table, self.C1_table, kind='linear', fill_value='extrapolate')(self.x_hat_state[0])
            C1 = np.interp(self.x_hat_state[0],self.SOC_table, self.C1_table);
            
        # State EKF - Prediction Step
        A_state = np.array([[1, 0],
                            [0, np.exp(-dt / (R1 * C1))]])
        x_hat_state_pred = np.zeros(2)
        x_hat_state_pred[0] = self.x_hat_state[0] - (dt / self.x_hat_param) * self.meas_curr_a_k_1
        x_hat_state_pred[1] = (np.exp(-dt / (R1 * C1)) * self.x_hat_state[1] +
                              R1 * (1 - np.exp(-dt / (R1 * C1))) * self.meas_curr_a_k_1)
        P_state_pred = A_state @ self.P_state @ A_state.T + self.Q_state

        # State EKF - Measurement Update
        if meas_curr_a < 0:  # Charging
            #Rs =interp1d(self.SOC_1_table, self.Rs_1_table, kind='linear', fill_value='extrapolate')(x_hat_state_pred[0])
            Rs = np.interp(x_hat_state_pred[0] ,self.SOC_1_table, self.Rs_1_table)
            #OCV_pred = interp1d(self.SOC_OCV, self.OCV_charge, kind='linear', fill_value='extrapolate')(x_hat_state_pred[0])
            OCV_pred =np.interp(x_hat_state_pred[0],self.SOC_OCV, self.OCV_charge)
            dOCV_dSOC_k = self.dOCV_dSOC_1(x_hat_state_pred[0])
            V_pred_state = OCV_pred - x_hat_state_pred[1] - Rs * meas_curr_a
            
        else:  # Discharging
            #Rs = interp1d(self.SOC_table, self.Rs_table, kind='linear', fill_value='extrapolate')(x_hat_state_pred[0])
            Rs = np.interp(x_hat_state_pred[0] ,self.SOC_table, self.Rs_table)
            #OCV_pred = interp1d(self.SOC_OCV, self.OCV_discharge, kind='linear', fill_value='extrapolate')(x_hat_state_pred[0])
            OCV_pred =np.interp(x_hat_state_pred[0],self.SOC_OCV, self.OCV_discharge)
            dOCV_dSOC_k = self.dOCV_dSOC(x_hat_state_pred[0])
            V_pred_state = OCV_pred - x_hat_state_pred[1] - Rs * meas_curr_a
            

        C_state = np.array([dOCV_dSOC_k, -1])
        K_state_previous = self.K_state.copy()
        denom = C_state @ P_state_pred @ C_state.T + self.R_state
        if abs(denom) < 1e-10:
            denom = 1e-10
        self.K_state = P_state_pred @ C_state.T / denom
        self.x_hat_state = x_hat_state_pred + self.K_state * (meas_volt_v - V_pred_state)
        self.P_state = P_state_pred - np.outer(self.K_state, C_state) @ P_state_pred

        # Parameter EKF - Intermediate
        C_param = dOCV_dSOC_k * dt * meas_curr_a / (self.x_hat_param**2) + np.array([dOCV_dSOC_k, 0]) @ self.dx_by_dtheta_k
        self.dx_by_dtheta_k = np.array([dt * self.meas_curr_a_k_1 / (self.x_hat_param**2), 0]) + A_state @ self.dx_by_dtheta_k_1
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
            if np.isnan(self.x_hat_param):
               self.x_hat_param = self.x_hat_param_k_1
               
            self.P_param = (1 - K_param * C_param) * P_param_pred
            # move all entries down to make room for new entry
            temp = self.ekf_cap_est_mah[0] #latest entry
            if temp < 0:
                temp = (1/3.6)*self.x_hat_param
            self.ekf_cap_est_mah[1:]=self.ekf_cap_est_mah[:-1]
            self.ekf_cap_est_mah[0] = (1-PARAMS.ALPHA_EKF)*temp + PARAMS.ALPHA_EKF*(1/3.6)*self.x_hat_param
            self.update_counter = 0
            
        self.x_hat_param_k_1 = self.x_hat_param    
        # all of these should be logged to the json for backup
        # Update outputs (states)
        self.est_soc = self.x_hat_state[0]
        self.est_volt_v = OCV_pred - self.x_hat_state[1] - Rs * meas_curr_a
        self.est_capacity_as = self.x_hat_param
        # Update outputs (parameters)
        self.est_cov_state = self.P_state
        self.est_cov_param = self.P_param
        self.meas_curr_a_k_1 = meas_curr_a 
        #self.K_state
        #self.x_hat_state
        #self.dx_by_dtheta_k
        
    @property
    def state_code(self):
        return {
            'CHG': 0,
            'DIS': 1,
            'REST': 2,
            'CHG_REST': 3,
            'DIS_REST': 4,
            'CHG_LOW': 5,
            'DIS_LOW': 6
            }.get(self.state, 255) # 255 = undefined
    
    @property
    def mode_code(self):
        return {
            'CYCLE': 0,
            'TEST': 1
            }.get(self.mode, 255)

    def update_predictions(self):
        #based on the history of capacity estimates, linearly interpolate predictions
        y = np.delete(self.ekf_cap_est_mah, np.where(self.ekf_cap_est_mah < 0))
        if y.size < 3:
            self.pred_ekf_one = 0
            self.pred_ekf_two = 0
        else:
            x = linspace(0, y.size-1, y.size)
            p = np.polyfit(x, y, 1)
            self.pred_ekf_one = polyval(p,  -600)
            self.pred_ekf_two = polyval(p, -1200)
        y = np.delete(self.cyc_cap_est_mah, np.where(self.cyc_cap_est_mah < 0))
        if y.size < 3:
            self.pred_cyc_one = 0
            self.pred_cyc_two = 0
        else:
            x = linspace(0, y.size-1, y.size)
            p = np.polyfit(x, y, 1)
            self.pred_cyc_one = polyval(p,  -180)
            self.pred_cyc_two = polyval(p,  -360)
    
    def backup(self):
        #save all the things to the json
        ch_vals = {"state" : self.state,
            "state_prev" : self.state_prev,
            "mode" : self.mode,
            "test_sequence" : self.test_sequence,
            "volt_v" : self.volt_v,     
            "temp_c" : self.temp_c,
            "cycle_count" : self.cycle_count,
            "chg_val" : self.chg_val,
            "dis_val" : self.dis_val,
            "chg_low_val" : self.chg_low_val,
            "dis_low_val" : self.dis_low_val,
            "pulse_state" : self.pulse_state,
            "en_chg_state" : self.en_chg_state,
            "en_dis_state" : self.en_dis_state,
            "en_cur_state" : self.en_cur_state,
            "update_act" : self.update_act,
            "cc_capacity_mas" : self.cc_capacity_mas,
            "cc_soc_mas" : self.cc_soc_mas,
            "R_SHUNT_OHMS" : self.R_SHUNT_OHMS,
            "est_capacity_as" : self.est_capacity_as,
            "est_soc" : self.est_soc,
            "est_cov_state00" : self.est_cov_state[0,0], 
            "est_cov_state01" : self.est_cov_state[0,1],
            "est_cov_state10" : self.est_cov_state[1,0],
            "est_cov_state11" : self.est_cov_state[1,1],
            "est_cov_param" : self.est_cov_param, } #covariance for param EKF

        json_str = json.dumps(ch_vals) #convert to json str
    
        with open(self.file_name, 'w') as json_out:
            json_out.write(json_str)