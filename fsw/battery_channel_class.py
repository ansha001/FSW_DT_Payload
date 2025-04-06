class battery_channel:
    def __init__(self, channel, state, mode, cycle_count, volt_v, temp_c, chg_val, dis_val):
        self.channel = channel
        self.state   = state
        self.state_prev = state
        self.mode    = mode
        self.test_sequence = 0
        self.volt_v  = volt_v
        self.temp_c  = temp_c
        self.curr_ma = 0
        self.cycle_count = cycle_count
        self.time_resting_started_s = -1
        self.time_prev_s = -1
        self.chg_val = chg_val
        self.dis_val = dis_val
        self.chg_low_val = chg_val
        self.dis_low_val = dis_val
        self.pulse_state  = False
        self.en_chg_state = False
        self.en_dis_state = False
        self.en_cur_state = False
        self.cc_capacity_mas = 0
        self.cc_soc_mas = 0
        self.est_soc = 0
        self.est_volt_v = 0
        self.est_cov = 0 #this will be a 2x2 matrix
        #TODO - add other state estimate variables
        
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
                self.test_sequence = 4
                self.time_resting_started_s = time_iter_s
            if self.test_sequence == 4 and time_iter_s - self.time_resting_started_s > PARAMS.TIME_PULSE_TEST_S:
                self.state = 'REST'
                self.test_sequence = 5
                self.time_resting_started_s = time_iter_s
            if self.test_sequence == 5 and time_iter_s - self.time_resting_started_s > PARAMS.TIME_PULSE_REST_S:
                self.state = 'CHG'
                self.test_sequence = 6
                self.time_resting_started_s = time_iter_s
            if self.test_sequence == 6 and time_iter_s - self.time_resting_started_s > PARAMS.TIME_PULSE_TEST_S:
                self.state = 'REST'
                self.test_sequence = 7
                self.time_resting_started_s = time_iter_s
            if self.test_sequence == 7 and time_iter_s - self.time_resting_started_s > PARAMS.TIME_PULSE_REST_S:
                self.state = 'DIS'
                self.test_sequence = 8
                self.time_prev_s = time_iter_s
            if self.test_sequence == 8 and self.cc_soc_mas >= PARAMS.SOC_PULSE1 * self.cc_capacity_mas and self.volt_v >= PARAMS.FB_PULSE1_V:
                self.cc_soc_mas = self.cc_soc_mas - self.curr_mas * (time_iter_s - self.time_prev_s)
                self.time_prev_s = time_iter_s
            if self.test_sequence == 8 and (self.cc_soc_mas < PARAMS.SOC_PULSE1 * self.cc_capcacity_mas or self.volt_v < PARAMS.FB_PULSE1_V):
                self.state = 'REST'
                self.test_sequence = 9
                self.time_resting_started_s = time_iter_s
            if self.test_sequence == 9 and time_iter_s - self.time_resting_started_s > PARAMS.TIME_TEST_REST_S:
                self.state = 'DIS'
                self.test_sequence = 10
                self.time_resting_started_s = time_iter_s
            if self.test_sequence == 10 and time_iter_s - self.time_resting_started_s > PARAMS.TIME_PULSE_TEST_S:
                self.state = 'REST'
                self.test_sequence = 11
                self.time_resting_started_s = time_iter_s
            if self.test_sequence == 11 and time_iter_s - self.time_resting_started_s > PARAMS.TIME_PULSE_REST_S:
                self.state = 'CHG'
                self.test_sequence = 12
                self.time_resting_started_s = time_iter_s
            if self.test_sequence == 12 and time_iter_s - self.time_resting_started_s > PARAMS.TIME_PULSE_TEST_S:
                self.state = 'REST'
                self.test_sequence = 13
                self.time_resting_started_s = time_iter_s
            if self.test_sequence == 13 and time_iter_s - self.time_resting_started_s > PARAMS.TIME_PULSE_REST_S:
                self.state = 'DIS'
                self.test_sequence = 14
                self.time_prev_s = time_iter_s
            if self.test_sequence == 14 and self.cc_soc_mas >= PARAMS.SOC_PULSE2 * self.cc_capacity_mas and self.volt_v >= PARAMS.FB_PULSE2_V:
                self.cc_soc_mas = self.cc_soc_mas - self.curr_mas * (time_iter_s - self.time_prev_s)
                self.time_prev_s = time_iter_s
            if self.test_sequence == 14 and (self.cc_soc_mas < PARAMS.SOC_PULSE2 * self.cc_capcacity_mas or self.volt_v < PARAMS.FB_PULSE2_V):
                self.state = 'REST'
                self.test_sequence = 15
                self.time_resting_started_s = time_iter_s
            if self.test_sequence == 15 and time_iter_s - self.time_resting_started_s > PARAM.TIME_TEST_REST_S:
                self.state = 'DIS'
                self.test_sequence = 16
                self.time_resting_started_s = time_iter_s
                if self.test_sequence == 16 and time_iter_s - self.time_resting_started_s > PARAMS.TIME_PULSE_TEST_S:
                self.state = 'REST'
                self.test_sequence = 17
                self.time_resting_started_s = time_iter_s
            if self.test_sequence == 17 and time_iter_s - self.time_resting_started_s > PARAMS.TIME_PULSE_REST_S:
                self.state = 'CHG'
                self.test_sequence = 18
                self.time_resting_started_s = time_iter_s
            if self.test_sequence == 18 and time_iter_s - self.time_resting_started_s > PARAMS.TIME_PULSE_TEST_S:
                self.state = 'REST'
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
        
        if self.state == 'CHG':     
            #determine charging current setpoint
            charge_setpoint_ma = 0
            if self.temp_c > PARAMS.TEMP_CHG_MIN_C:
                charge_setpoint_ma = PARAMS.CHG_MIN_SETPT_MA
            if self.temp_c > PARAMS.TEMP_CHG_LOWER_C:
                charge_setpoint_ma = PARAMS.CHG_LOWER_SETPT_MA
            if self.temp_c > PARAMS.TEMP_CHG_UPPER_C:
                charge_setpoint_ma = PARAMS.CHG_UPPER_SETPT_MA
            if self.temp_c > PARAMS.TEMP_CHG_MAX_C:
                charge_setpoint_ma = 0
            
            if charge_setpoint_ma ~= 0:
                if self.curr_ma < charge_setpoint_ma - PARAMS.CHG_SETPT_DELTA_MA:
                    self.chg_val += 1
                    self.chg_val = min(1024, self.chg_val)
                elif self.curr_ma > charge_setpoint_ma + PARAMS.CHG_SETPT_DELTA_MA:
                    self.chg_val -= 1
                    self.chg_val = max(0, self.chg_val)
        
        if self.state == 'CHG_LOW':
            charge_setpoint_ma = 0
            if self.temp_c > PARAMS.TEMP_CHG_MIN_C and self.temp_c < PARAMS.TEMP_CHG_MAX_C:
                charge_setpoint_ma = PARAMS.CHG_LOW_SETPT_MA
                if self.curr_ma < charge_setpoint_ma - PARAMS.CHG_SETPT_DELTA_MA:
                    self.chg_low_val += 1
                    self.chg_low_val = min(1024, self.chg_low_val)
                elif self.curr_ma > charge_setpoint_ma + PARAMS.CHG_SETPT_DELTA_MA:
                    self.chg_low_val -= 1
                    self.chg_low_val = max(0, self.chg_low_val)
        
        if self.state == 'DIS':
            discharge_setpoint_ma = 0
            if self.temp_c > PARAMS.TEMP_MIN_C and self.temp_c < PARAMS.TEMP_MAX_C:
                discharge_setpoint_ma = PARAMS.DIS_SETPT_MA
                if self.curr_ma < discharge_setpoint_ma - PARAMS.DIS_SETPT_DELTA_MA:
                    self.dis_val += 1
                    self.dis_val = min(256, self.dis_val)
                elif self.curr_ma > discharge_setpoint_ma + PARAMS.DIS_SETPT_DELTA_MA:
                    self.dis_val -= 1
                    self.dis_val = max(0, self.dis_val)
                
        if self.state == 'DIS_LOW':
            discharge_setpoint_ma = 0
            if self.temp_c > PARAMS.TEMP_MIN_C and self.temp_c < PARAMS.TEMP_MAX_C:
                discharge_setpoint_ma = PARAMS.DIS_LOW_SETPT_MA
                if self.curr_ma < discharge_setpoint_ma - PARAMS.DIS_SETPT_DELTA_MA:
                    self.dis_val += 1
                    self.dis_val = min(256, self.dis_val)
                elif self.curr_ma > discharge_setpoint_ma + PARAMS.DIS_SETPT_DELTA_MA:
                    self.dis_val -= 1
                    self.dis_val = max(0, self.dis_val)
        
        self.en_cur_state = False
        self.en_dis_state = False
        self.en_chg_state = False
        if discharge_setpoint_ma > 0:
            self.en_dis_state = True
            if discharge_setpoint_ma > PARAMS.DIS_TRANS_MA:  # TODO revisit transition value
                self.en_cur_state = True
        if charge_setpoint_ma < 0:
            self.en_chg_state = True
            
    def state_estimate_fast(self, meas_volt_v, meas_curr_ma):
        #TODO implement state estimator
        self.soc = 0
    
    def state_estimate_slow(self, meas_volt_v, meas_curr_ma):
        #TODO implement state estimator
        self.soc = 0