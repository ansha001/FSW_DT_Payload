import numpy as np
import json
from scipy.interpolate import interp1d
from scipy.io import loadmat  # Load the MATLAB data files

class battery_channel:
    def __init__(self, channel, state, mode, cycle_count, volt_v, temp_c, chg_val, dis_val, file_name, ekf_cap_file, cyc_cap_file, PARAMS):
        self.file_name = file_name
        self.channel = channel
        self.curr_ma = 0
        self.meas_curr_a_k_1 = 0
        self.time_resting_started_s = -1
        self.time_prev_s = -1
        self.time_iter_prev_s = -1
        self.update_act = False
        self.R_state = PARAMS.R_state  # 0.015
        self.Q_state = PARAMS.Q_state  # np.diag([1e-6, 1e-3])
        self.Q_param = PARAMS.Q_param  # 1e-1
        self.R_param = PARAMS.R_param  # 3.72725e-3
        self.update_counter = 0
        self.update_counter_beta = 0  # Already present, used for beta updates
        self.est_volt_v = 0
        self.pred_cyc_one = 0
        self.pred_cyc_two = 0
        self.pred_ekf_one = 0
        self.pred_ekf_two = 0

        # New variables for beta estimation and capacity forecast
        self.betaWindow = 100000  # Iteration count equivalent to 1e5 seconds
        self.forecastHorizon = 6e5  # Seconds, kept for forecast calculation
        self.anchorCount = None  # Counter value at last beta update
        self.anchorCap = None  # Capacity at last beta update
        self.nextBetaCount = None  # Counter value for next beta update
        self.beta_curr = 0  # Current beta (capacity change per iteration)
        self.capPred = None  # Forecasted capacity 6e5 seconds ahead

        # Load cap estimate backups
        try:
            temp = np.load(ekf_cap_file)
            self.ekf_cap_est_mah = temp[:, channel]
            temp = np.load(cyc_cap_file)
            self.cyc_cap_est_mah = temp[:, channel]
        except Exception:
            print('Error reading capacity files')
            self.ekf_cap_est_mah = -1 * np.ones([1200])
            self.cyc_cap_est_mah = -1 * np.ones([360])

        # Load from JSON if available
        try:
            with open(file_name, 'r') as json_in:
                vals = json.loads(json_in.read())
                self.state = vals["state"]
                self.state_prev = vals["state_prev"]
                self.mode = vals["mode"]
                self.test_sequence = vals["test_sequence"]
                self.total_cycles = vals["total_cycles"]
                self.total_rpts = vals["total_rpts"]
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
                self.last_cyc_cap_mah = vals["last_cyc_cap_mah"]
                self.R_SHUNT_OHMS = vals["R_SHUNT_OHMS"]
                self.est_capacity_as = vals["est_capacity_as"]
                self.est_soc = vals["est_soc"]
                self.est_cov_state = np.zeros((2, 2))
                self.est_cov_state[0,0] = vals["est_cov_state00"]
                self.est_cov_state[0,1] = vals["est_cov_state01"]
                self.est_cov_state[1,0] = vals["est_cov_state10"]
                self.est_cov_state[1,1] = vals["est_cov_state11"]
                self.est_cov_param = vals["est_cov_param"]
                self.P_state = self.est_cov_state
                self.P_param = self.est_cov_param

                # Load new beta and forecast variables
                self.anchorCount = vals.get("anchorCount", None)
                if self.anchorCount == -1:
                    self.anchorCount = None
                self.anchorCap = vals.get("anchorCap", None)
                if self.anchorCap == -1:
                    self.anchorCap = None
                self.nextBetaCount = vals.get("nextBetaCount", None)
                if self.nextBetaCount == -1:
                    self.nextBetaCount = None
                self.beta_curr = vals.get("beta_curr", 0)
                self.capPred = vals.get("capPred", None)
                if self.capPred == -1:
                    self.capPred = None
        except Exception:
            print('Error reading battery json')
            # Initialize default values if JSON read fails
            self.state = state
            self.state_prev = state
            self.mode = mode
            self.test_sequence = 0
            self.total_cycles = 0
            self.total_rpts = 0
            self.volt_v = volt_v
            self.temp_c = temp_c
            self.cycle_count = cycle_count
            self.chg_val = chg_val
            self.dis_val = dis_val
            self.chg_low_val = 200
            self.dis_low_val = dis_val
            self.pulse_state = False
            self.en_chg_state = False
            self.en_dis_state = False
            self.en_cur_state = False
            self.cc_capacity_mas = 0
            self.cc_soc_mas = 0
            self.last_cyc_cap_mah = -1
            self.R_SHUNT_OHMS = 1.0
            self.est_capacity_as = 0.0476705 * 3600
            self.est_soc = 0
            self.est_cov_state = np.zeros((2, 2))
            self.est_cov_param = -1
            self.P_state = np.diag([1e-7, 1e-7])
            self.P_param = 1e-1

        self.x_hat_state = np.array([self.est_soc, self.est_volt_v])
        self.x_hat_param = self.est_capacity_as
        self.x_hat_param_k_1 = self.est_capacity_as

        self.K_state = np.zeros(2)
        self.dx_by_dtheta_k = np.zeros(2)
        self.dx_by_dtheta_k_1 = np.zeros(2)

        # Lookup tables (unchanged)
        rc_rs_values = loadmat(r'mat_files/RC_Rs_values_1.mat')
        rc_rs_values_charging = loadmat(r'mat_files/RC_Rs_values_1_charging.mat')
        ocv_data = loadmat(r'mat_files/LIR2032_EEMB_Cell1_25C_OCV.mat')
        data_cell2 = loadmat(r'mat_files/data_Cell2_25C.mat')
        self.lookup_table = rc_rs_values['lookupTable']
        self.SOC_table = self.lookup_table['SOC'][0][0].flatten()
        self.Rs_table = self.lookup_table['Rs'][0][0].flatten()
        self.R1_table = self.lookup_table['R1'][0][0].flatten()
        self.C1_table = self.lookup_table['C1'][0][0].flatten()
        self.lookup_table_1 = rc_rs_values_charging['lookupTable_1'][0][0]
        self.SOC_1_table = self.lookup_table_1['SOC'][0][0].flatten()
        self.Rs_1_table = self.lookup_table_1['Rs'][0][0].flatten()
        self.R1_1_table = self.lookup_table_1['R1'][0][0].flatten()
        self.C1_1_table = self.lookup_table_1['C1'][0][0].flatten()
        self.SOC_OCV = ocv_data['SOC'].flatten()
        self.OCV_charge = ocv_data['OCV_charge'].flatten()
        self.OCV_discharge = ocv_data['OCV_discharge'].flatten()
        degree = 5
        self.coefficients_1 = np.polyfit(self.SOC_OCV, self.OCV_charge, degree)
        self.coefficients = np.polyfit(self.SOC_OCV, self.OCV_discharge, degree)
        self.derivative_coefficients_1 = np.polyder(self.coefficients_1)
        self.derivative_coefficients = np.polyder(self.coefficients)

    

    def state_estimate(self, meas_volt_v, meas_curr_ma, time_iter_s, PARAMS):
        meas_curr_a = meas_curr_ma / 1000

        # Compute time step
        dt = time_iter_s - self.time_iter_prev_s
        if self.time_iter_prev_s < 0:
            dt = 1
        dt = max(0.01, min(10, dt))
        self.time_iter_prev_s = time_iter_s

        # State EKF - Prediction and Update (unchanged)
        if self.meas_curr_a_k_1 < 0:
            R1 = np.interp(self.x_hat_state[0], self.SOC_1_table, self.R1_1_table)
            C1 = np.interp(self.x_hat_state[0], self.SOC_1_table, self.C1_1_table)
        else:
            R1 = np.interp(self.x_hat_state[0], self.SOC_table, self.R1_table)
            C1 = np.interp(self.x_hat_state[0], self.SOC_table, self.C1_table)

        A_state = np.array([[1, 0], [0, np.exp(-dt / (R1 * C1))]])
        x_hat_state_pred = np.zeros(2)
        x_hat_state_pred[0] = self.x_hat_state[0] - (dt / self.x_hat_param) * self.meas_curr_a_k_1
        x_hat_state_pred[1] = (np.exp(-dt / (R1 * C1)) * self.x_hat_state[1] +
                               R1 * (1 - np.exp(-dt / (R1 * C1))) * self.meas_curr_a_k_1)
        P_state_pred = A_state @ self.P_state @ A_state.T + self.Q_state

        if meas_curr_a < 0:
            Rs = np.interp(x_hat_state_pred[0], self.SOC_1_table, self.Rs_1_table)
            OCV_pred = np.interp(x_hat_state_pred[0], self.SOC_OCV, self.OCV_charge)
            dOCV_dSOC_k = self.dOCV_dSOC_1(x_hat_state_pred[0])
            V_pred_state = OCV_pred - x_hat_state_pred[1] - Rs * meas_curr_a
        else:
            Rs = np.interp(x_hat_state_pred[0], self.SOC_table, self.Rs_table)
            OCV_pred = np.interp(x_hat_state_pred[0], self.SOC_OCV, self.OCV_discharge)
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
        self.update_counter_beta += 1
        if self.update_counter >= 1000:
            P_param_pred = self.P_param + self.Q_param
            denom_param = C_param * P_param_pred * C_param + self.R_param
            if abs(denom_param) < 1e-10:
                denom_param = 1e-10
            K_param = P_param_pred * C_param / denom_param
            self.x_hat_param = self.x_hat_param + K_param * (meas_volt_v - V_pred_state)
            if np.isnan(self.x_hat_param):
                self.x_hat_param = self.x_hat_param_k_1
            self.P_param = (1 - K_param * C_param) * P_param_pred
            temp = self.ekf_cap_est_mah[0]
            if temp < 0:
                temp = (1/3.6) * self.x_hat_param
            self.ekf_cap_est_mah[1:] = self.ekf_cap_est_mah[:-1]
            self.ekf_cap_est_mah[0] = (1 - PARAMS.ALPHA_EKF) * temp + PARAMS.ALPHA_EKF * (1/3.6) * self.x_hat_param
            self.update_counter = 0

        # Beta Estimation and Capacity Forecast
        if self.anchorCount is None:
            #self.anchorCount = self.update_counter_beta
            self.anchorCap = self.x_hat_param
            self.nextBetaCount = self.anchorCount + self.betaWindow
            self.capPred = self.x_hat_param  # Initial forecast
        else:
            if self.update_counter_beta >= self.nextBetaCount:
                delta_count = self.update_counter_beta - self.anchorCount
                if delta_count > 0:
                    # Estimate beta_curr as capacity change per iteration
                    self.beta_curr = (self.x_hat_param - self.anchorCap) / delta_count  # Amp*secs/secs
                else:
                    self.beta_curr = 0
                self.anchorCap = self.x_hat_param
                self.anchorCount = self.update_counter_beta
                self.nextBetaCount = self.update_counter_beta + self.betaWindow
                # Adjust beta_curr to As/s and compute forecast
                dt_avg = dt  # Use current dt as an estimate
                beta_curr_as_s = self.beta_curr / dt_avg  # Convert to As/s
                self.capPred = self.anchorCap + beta_curr_as_s * self.forecastHorizon

        self.x_hat_param_k_1 = self.x_hat_param
        self.est_soc = self.x_hat_state[0]
        self.est_volt_v = OCV_pred - self.x_hat_state[1] - Rs * meas_curr_a
        self.est_capacity_as = self.x_hat_param
        self.est_cov_state = self.P_state
        self.est_cov_param = self.P_param
        self.meas_curr_a_k_1 = meas_curr_a

    def backup(self):
        ch_vals = {
            "state": self.state,
            "state_prev": self.state_prev,
            "mode": self.mode,
            "test_sequence": self.test_sequence,
            "total_cycles": self.total_cycles,
            "total_rpts": self.total_rpts,
            "volt_v": self.volt_v,
            "temp_c": self.temp_c,
            "cycle_count": self.cycle_count,
            "chg_val": self.chg_val,
            "dis_val": self.dis_val,
            "chg_low_val": self.chg_low_val,
            "dis_low_val": self.dis_low_val,
            "pulse_state": self.pulse_state,
            "en_chg_state": self.en_chg_state,
            "en_dis_state": self.en_dis_state,
            "en_cur_state": self.en_cur_state,
            "update_act": self.update_act,
            "cc_capacity_mas": self.cc_capacity_mas,
            "cc_soc_mas": self.cc_soc_mas,
            "last_cyc_cap_mah": self.last_cyc_cap_mah,
            "R_SHUNT_OHMS": self.R_SHUNT_OHMS,
            "est_capacity_as": self.est_capacity_as,
            "est_soc": self.est_soc,
            "est_cov_state00": self.est_cov_state[0,0],
            "est_cov_state01": self.est_cov_state[0,1],
            "est_cov_state10": self.est_cov_state[1,0],
            "est_cov_state11": self.est_cov_state[1,1],
            "est_cov_param": self.est_cov_param,
            # New beta and forecast variables
            "anchorCount": self.anchorCount if self.anchorCount is not None else -1,
            "anchorCap": self.anchorCap if self.anchorCap is not None else -1,
            "nextBetaCount": self.nextBetaCount if self.nextBetaCount is not None else -1,
            "beta_curr": self.beta_curr,
            "capPred": self.capPred if self.capPred is not None else -1
        }
        
        json_str = json.dumps(ch_vals)

        with open(self.file_name, 'w') as json_out:
            json_out.write(json_str)

    