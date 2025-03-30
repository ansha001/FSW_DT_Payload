class battery_channel:
    def __init__(self, channel, state, mode, cycle_count, volt_v, temp_c, chg_val, dis_val):
        self.channel = channel
        self.state   = state
        self.state_prev = state
        self.mode    = mode
        self.volt_v  = volt_v
        self.temp_c  = temp_c
        self.curr_ma = 0
        self.cycle_count = cycle_count
        self.time_resting_started_s = -1
        self.chg_val = chg_val
        self.dis_val = dis_val
        self.en_chg_state = False
        self.en_dis_state = False
        self.en_cur_state = False
        
    def channel_logic(self, time_iter_s, PARAMS):
        if self.cycle_count < 20:
            self.mode = 'CYCLE'
            if self.state == 'CHG' and self.volt_v >= PARAMS.CHG_LIMIT_V:
                self.state = 'CHG_REST'
                self.time_resting_started_s = time_iter_s
            if self.state == 'DIS' and self.volt_v <= PARAMS.DIS_LIMIT_V:
                self.state = 'DIS_REST'
                self.time_resting_started_s = time_iter_s
            if self.state == 'CHG_REST' and time_iter_s - self.time_resting_started_s > PARAMS.TIME_TO_REST_S:
                self.state = 'DIS'
            if self.state == 'DIS_REST' and time_iter_s - self.time_resting_started_s > PARAMS.TIME_TO_REST_S:
                self.state = 'CHG'
                self.cycle_count += 1
        else:
            self.mode = 'TEST'
            self.state = 'CHG_REST'
            #TODO - define RPT logic sequence
    
    def channel_action(self, PARAMS):
        charge_setpoint_ma = 0
        discharge_setpoint_ma = 0
        
        if self.mode == 'CYCLE' and self.state == 'CHG':     
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
            
            if self.curr_ma < charge_setpoint_ma - PARAMS.CHG_SETPT_DELTA_MA:
                self.chg_val += 1
            elif self.curr_ma > charge_setpoint_ma + PARAMS.CHG_SETPT_DELTA_MA:
                self.chg_val -= 1
            self.chg_val = max(0, min(1024, self.chg_val))
                
        if self.mode == 'CYCLE' and self.state == 'DIS':
            discharge_setpoint_ma = 0
            if self.temp_c > PARAMS.TEMP_MIN_C:
                discharge_setpoint_ma = PARAMS.DIS_SETPT_MA
            if self.temp_c > PARAMS.TEMP_MAX_C:
                discharge_setpoint_ma = 0
            
            if self.curr_ma < discharge_setpoint_ma - PARAMS.DIS_SETPT_DELTA_MA:
                self.dis_val += 1
                self.dis_val = min(256, self.dis_val)
            elif self.curr_ma > discharge_setpoint_ma + PARAMS.DIS_SETPT_DELTA_MA:
                self.dis_val -= 1
                self.dis_val = max(0, self.dis_val)
                
        if self.mode == 'TEST':
            # do characterization test things
            #TODO implement characterization test
            self.cycle_count = 0
            self.mode = 'CYCLE'
            self.state = 'CHG'
            #safe_board(self.channel)
        
        self.en_cur_state = False
        self.en_dis_state = False
        self.en_chg_state = False
        if discharge_setpoint_ma > 0:
            self.en_dis_state = True
            if discharge_setpoint_ma > PARAMS.DIS_TRANS_MA:  # TODO revisit transition value
                self.en_cur_state = True
        if charge_setpoint_ma < 0:
            self.en_chg_state = True