import numpy as np

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
        self.est_soc = 0
        self.est_volt_v = 0
        self.est_cov = np.zeros((2, 2))  # 2x2 covariance matrix for state EKF

        # EKF state variables
        self.x_hat_state = np.array([1.0, 0.0])  # [SOC, Vc1]
        self.P_state = np.diag([1e-7, 1e-7])     # State covariance
        self.Q_state = np.diag([1e-6, 1e-3])     # Process noise covariance
        self.R_state = 0.015                     # Measurement noise covariance

        # Parameter EKF variables
        self.x_hat_param = 0.0                   # Estimated capacity (to be initialized)
        self.P_param = 1e-1                      # Parameter covariance
        self.Q_param = 1e-1                      # Parameter process noise
        self.R_param = 3.72725e-3                # Parameter measurement noise
        self.dx_by_dtheta_k = np.zeros(2)        # Sensitivity vector
        self.dx_by_dtheta_k_1 = np.zeros(2)      # Previous sensitivity vector
        self.K_state = np.zeros(2)               # State Kalman gain

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
            R1 = np.interp(self.x_hat_state[0], self.SOC_1_table, self.R1_1_table)
            C1 = np.interp(self.x_hat_state[0], self.SOC_1_table, self.C1_1_table)
        else:  # Discharging
            R1 = np.interp(self.x_hat_state[0], self.SOC_table, self.R1_table)
            C1 = np.interp(self.x_hat_state[0], self.SOC_table, self.C1_table)

        # State EKF - Prediction
        tau = R1 * C1
        A_state = np.array([[1, 0],
                            [0, np.exp(-dt / tau)]])
        x_hat_state_pred = np.zeros(2)
        x_hat_state_pred[0] = self.x_hat_state[0] - (dt / self.x_hat_param) * meas_curr_ma
        x_hat_state_pred[1] = (np.exp(-dt / tau) * self.x_hat_state[1] +
                              R1 * (1 - np.exp(-dt / tau)) * meas_curr_ma)
        P_state_pred = A_state @ self.P_state @ A_state.T + self.Q_state

        # State EKF - Measurement Update
        if meas_curr_ma < 0:  # Charging
            Rs = np.interp(x_hat_state_pred[0], self.SOC_1_table, self.Rs_1_table)
            OCV_pred = np.interp(x_hat_state_pred[0], self.SOC_OCV, self.OCV_charge)
            dOCV_dSOC_k = np.polyval(self.derivative_coefficients_1, x_hat_state_pred[0])
        else:  # Discharging
            Rs = np.interp(x_hat_state_pred[0], self.SOC_table, self.Rs_table)
            OCV_pred = np.interp(x_hat_state_pred[0], self.SOC_OCV, self.OCV_discharge)
            dOCV_dSOC_k = np.polyval(self.derivative_coefficients, x_hat_state_pred[0])

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
        C_param = (dOCV_dSOC_k * dt * meas_curr_ma / (self.x_hat_param**2) +
                   np.array([dOCV_dSOC_k, 0]) @ self.dx_by_dtheta_k)
        self.dx_by_dtheta_k = (np.array([dt * meas_curr_ma / (self.x_hat_param**2), 0]) +
                              A_state @ self.dx_by_dtheta_k_1)
        self.dx_by_dtheta_k_1 = self.dx_by_dtheta_k - K_state_previous * C_param

        # Parameter EKF - Update (every 1000 iterations)
        self.update_counter = getattr(self, 'update_counter', 0) + 1
        if self.update_counter % 1000 == 0:
            P_param_pred = self.P_param + self.Q_param
            denom_param = C_param * P_param_pred * C_param + self.R_param
            if abs(denom_param) < 1e-10:
                denom_param = 1e-10
            K_param = P_param_pred * C_param / denom_param
            self.x_hat_param = self.x_hat_param + K_param * (meas_volt_v - V_pred_state)
            self.P_param = (1 - K_param * C_param) * P_param_pred
            self.update_counter = 0

        # Update outputs
        self.est_soc = self.x_hat_state[0]
        self.est_volt_v = V_pred_state
        self.est_cov = self.P_state
        self.cc_capacity_mas = self.x_hat_param  # Update capacity estimate

