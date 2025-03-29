class battery_channel:
    def __init__(self, channel, state, mode, cycle_count, volt_v, temp_c, EN_CHG_GPIO, EN_DIS_GPIO, POT_REG):
        self.channel = channel
        self.state   = state
        self.mode    = mode
        self.volt_v  = volt_v
        self.temp_c  = temp_c
        self.curr_ma = 0
        self.cycle_count = cycle_count
        self.time_resting_started_s = -1
        self.chg_val = 9
        self.dis_val = 76
        self.EN_CHG_GPIO = EN_CHG_GPIO
        self.EN_DIS_GPIO = EN_DIS_GPIO
        self.POT_REG     = POT_REG
        
#     def set_GPIO(cell_num, state, GPIO_LIST):
#         # manipulate EN_CHG GPIO pins according to cell num and desired state
#         if state=='ON' or state=='HIGH':
#             GPIO.output(GPIO_LIST[cell_num], GPIO.HIGH) #set HIGH
#         else:
#             GPIO.output(GPIO_LIST[cell_num], GPIO.LOW) #set LOW
            
    def set_POT(channel, val, ADDRS):
        # given "10-bit" potentiometer value
        # set each wiper in the potentiometer to the corresponding value
        val = max(0, min(1024, val))
        a = int(val/4)
        b = a + (val%4 > 2)
        c = a + (val%4 > 1)
        d = a + (val%4 > 0)
        set_wiper(ADDRS.POT_ADDRS[channel], ADDRS.POT_REGS[0], a, mode="block")
        set_wiper(ADDRS.POT_ADDRS[channel], ADDRS.POT_REGS[1], b, mode="block")
        set_wiper(ADDRS.POT_ADDRS[channel], ADDRS.POT_REGS[2], c, mode="block")
        set_wiper(ADDRS.POT_ADDRS[channel], ADDRS.POT_REGS[3], d, mode="block")
        
    def set_wiper(i2c_address, wiper_register, value, mode="byte"):
        if value < 0 or value > 256:
            raise ValueError("Wiper value out of range (0-256)")

        command_byte = (wiper_register << 4) | ((value >> 8) & 0x01)  
        lsb = value & 0xFF  

        try:
            if mode == "byte":
                bus.write_byte_data(i2c_address, wiper_register, lsb)
            elif mode == "block":
                bus.write_i2c_block_data(i2c_address, command_byte, [lsb])
            else:
                raise ValueError("invalid mode")
        except OSError as e:
            print(f"Error setting wiper {wiper_register}: {e}")
            return -1
    
    def read_wiper(i2c_address, wiper_register, mode="byte"):
        try:
            if mode == "byte":
                data = bus.read_byte_data(i2c_address, wiper_register)
                return data
            elif mode == "block":
                command_byte = (wiper_register << 4) | (0x03 << 2)  # read command 11
                data = bus.read_i2c_block_data(i2c_address, command_byte, 2)
                value = ((data[0] & 0x01) << 8) | data[1]  # combine MSB and LSB for 9bit value
                return value
            else:
                raise ValueError("invalid mode")
        except OSError as e:
            print(f"Error reading wiper {wiper_register}: {e}")
            return -1
    
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
    
    def channel_action(self, PARAMS, ADDRS):
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
                set_POT(self.channel, self.chg_val, ADDRS)
            elif self.curr_ma > charge_setpoint_ma + PARAMS.CHG_SETPT_DELTA_MA:
                self.chg_val -= 1
                set_POT(self.channel, self.chg_val, ADDRS)
            self.chg_val = max(0, min(1024, self.chg_val))
            
            if charge_setpoint_ma < 0:
                GPIO.output(self.EN_CHG_GPIO, GPIO.HIGH)
            else:
                GPIO.output(self.EN_CHG_GPIO, GPIO.LOW)
                
        elif self.mode[i] == 'CYCLE' and self.state == 'DIS':
            discharge_setpoint = PARAMS.DIS_SETPT
            
            if self.curr_ma < discharge_setpoint - PARAMS.DIS_SETPT_DELTA_MA:
                self.dis_val += 1
                self.dis_val = min(256, self.dis_val)
                set_wiper(POT_ADDRS[3], self.POT_REG, self.dis_val, mode="block")
            elif self.curr_ma > discharge_setpoint + PARAMS.DIS_SETPT_DELTA_MA:
                self.dis_val -= 1
                self.dis_val = max(0, self.dis_val)
                set_wiper(POT_ADDRS[3], self.POT_REG, self.dis_val, mode="block")
            GPIO.output(ADDRS.EN_DIS_GPIO_LIST[self.channel], GPIO.HIGH)
            GPIO.output(ADDRS.EN_CUR_GPIO_LIST[self.channel], GPIO.HIGH)
            
        elif self.mode == 'TEST':
            # do characterization test things
            #TODO implement characterization test
            self.cycle_count = 0
            self.mode = 'CYCLE'
            self.state = 'CHG'
            safe_board(self.channel)