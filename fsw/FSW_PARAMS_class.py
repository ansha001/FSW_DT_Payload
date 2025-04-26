import json
import numpy as np

class FSW_PARAMS:
    def __init__(self, file_name):
        #TODO - make initialization read from json or similar
        print('reading params')
        try:
            with open(file_name, 'r') as json_in:
                vals = json.loads(json_in.read())
                self.DT_CHECK_S   = vals["DT_CHECK_S"]             # time step between important checks
                self.DT_SENSORS_S = vals["DT_SENSORS_S"]           # time step between reading sensors
                self.DT_LOG_S     = vals["DT_LOG_S"]               # time step between logs
                self.DT_LOG2_S    = vals["DT_LOG2_S"]              # time step between type 2 logs
                self.DT_LOG3_S    = vals["DT_LOG3_S"]              # time step between type 3 logs
                self.DT_HEAT_S    = vals["DT_HEAT_S"]              # time step between checking heater
                self.DT_FAST_S    = vals["DT_FAST_S"]              # time step between fast loop of EKF
                self.DT_SLOW_S    = vals["DT_SLOW_S"]              # time step between slow loop of EKF
                self.DT_BACKUP_S  = vals["DT_BACKUP_S"]            # time step between backups
                self.CHG_LIMIT_V = vals["CHG_LIMIT_V"]             # voltage to stop charging
                self.DIS_LIMIT_V = vals["DIS_LIMIT_V"]             # voltage to stop discharging
                self.FB_PULSE1_V = vals["FB_PULSE1_V"]             # fallback voltage for 65% SOC
                self.FB_PULSE2_V = vals["FB_PULSE2_V"]             # fallback voltage for 30% SOC
                self.SOC_PULSE1  = vals["SOC_PULSE1"]              # SOC to perform pulse 1
                self.SOC_PULSE2  = vals["SOC_PULSE2"]              # SOC to perform pulse 2
                self.TIME_CYCLE_REST_S = vals["TIME_CYCLE_REST_S"] # time to rest between charge/discharge cycles, seconds  [NOMINALLY  5 MINUTES]
                self.TIME_PULSE_REST_S = vals["TIME_PULSE_REST_S"] # time to rest after pulse tests, seconds                [NOMINALLY 10 MINUTES]
                self.TIME_PULSE_TEST_S = vals["TIME_PULSE_TEST_S"] # time to hold current during pulse tests, seconds       [NOMINALLY 10 SECONDS]
                self.TIME_TEST_REST_S  = vals["TIME_TEST_REST_S"]  # time to rest before and after char test, seconds       [NOMINALLY  1 HOUR]
                self.TEMP_MAX_C = vals["TEMP_MAX_C"]               # above this, go to safe mode
                self.TEMP_MIN_C = vals["TEMP_MIN_C"]               # below this, go to safe mode
                self.TEMP_HEATER_ON_C  = vals["TEMP_HEATER_ON_C"]  # below this, turn heater on
                self.TEMP_HEATER_OFF_C = vals["TEMP_HEATER_OFF_C"] # above this, turn heater off
                self.TEMP_CHG_MAX_C    = vals["TEMP_CHG_MAX_C"]    # above this, stop charging
                self.TEMP_CHG_UPPER_C  = vals["TEMP_CHG_UPPER_C"]  # above this, charge setpoint is -40 mA
                self.TEMP_CHG_LOWER_C  = vals["TEMP_CHG_LOWER_C"]  # above this, charge setpoint is -10 mA
                self.TEMP_CHG_MIN_C    = vals["TEMP_CHG_MIN_C"]    # below this, stop charging
                
                self.CHG_UPPER_SETPT_MA = vals["CHG_UPPER_SETPT_MA"]   # nominal setpoint
                self.CHG_LOWER_SETPT_MA = vals["CHG_LOWER_SETPT_MA"]   # cold setpoint
                self.CHG_MIN_SETPT_MA   = vals["CHG_MIN_SETPT_MA"]     # minimum setpoint
                
                self.CHG_SETPT_DELTA_MA = vals["CHG_SETPT_DELTA_MA"]   # if charging current is outside bound, increment potentiometer
                self.CHG_LOW_SETPT_MA   = vals["CHG_LOW_SETPT_MA"]     # setpoint for 1/10 C charging
                self.CHG_LOW_SETPT_DELTA_MA = vals["CHG_LOW_SETPT_DELTA_MA"] #bound for when in low charge mode
                self.DIS_SETPT_MA       = vals["DIS_SETPT_MA"]         # discharge setpoint
                self.DIS_SETPT_DELTA_MA = vals["DIS_SETPT_DELTA_MA"]
                self.DIS_LOW_SETPT_MA   = vals["DIS_LOW_SETPT_MA"]     # setpoint for 1/10 C discharging
                self.DIS_LOW_SETPT_DELTA_MA = vals["DIS_LOW_SETPT_DELTA_MA"] #bound for when in low discharge mode
                
                self.DIS_TRANS_MA = vals["DIS_TRANS_MA"] #current to transition to low current mode
                self.CHG_VAL_INIT = vals["CHG_VAL_INIT"]
                self.DIS_VAL_INIT = vals["DIS_VAL_INIT"]
                self.NUM_CYCLES_PER_TEST = vals["NUM_CYCLES_PER_TEST"]    # typically 20
                self.num_boots = vals["num_boots"]
                self.ALPHA_EKF = vals["ALPHA_EKF"]
                self.ALPHA_CYC = vals["ALPHA_CYC"]
                
                self.R_state = vals["R_state"] #0.015                     # Measurement noise covariance
                Q1 = vals["Q_state1"]
                Q2 = vals["Q_state2"]
                self.Q_state = np.diag([Q1, Q2])     # Process noise covariance
                self.Q_param = vals["Q_param"] #1e-1                      # Parameter process noise
                self.R_param = vals["R_param"] #3.72725e-3                # Parameter measurement noise
        except Exception:
            print("Issue reading parameters")
            self.DT_CHECK_S = 0.2          # time step between important checks
            self.DT_SENSORS_S = 0.1         # time step between reading sensors
            self.DT_LOG_S  = 1.0            # time step between logs
            self.DT_LOG2_S = 240             # time step between type 2 logs
            self.DT_LOG3_S = 720            # time step between type 3 logs
            self.DT_HEAT_S = 180            # time step between checking heater
            self.DT_FAST_S = 1              # time step between fast loop of EKF
            self.DT_SLOW_S = 1000           # time step between slow loop of EKF
            self.DT_BACKUP_S = 60
            self.CHG_LIMIT_V = 4.20         # voltage to stop charging
            self.DIS_LIMIT_V = 2.75         # voltage to stop discharging
            self.FB_PULSE1_V = 3.10         # fallback voltage for 65% SOC
            self.FB_PULSE2_V = 3.00         # fallback voltage for 30% SOC
            self.SOC_PULSE1 = 0.65          # SOC to perform pulse 1
            self.SOC_PULSE2 = 0.30          # SOC to perform pulse 2
            self.TIME_CYCLE_REST_S = 300    # time to rest between charge/discharge cycles, seconds  [NOMINALLY  5 MINUTES]
            self.TIME_PULSE_REST_S = 600    # time to rest after pulse tests, seconds                [NOMINALLY 10 MINUTES]
            self.TIME_PULSE_TEST_S = 10     # time to hold current during pulse tests, seconds       [NOMINALLY 10 SECONDS]
            self.TIME_TEST_REST_S = 3600    # time to rest before and after char test, seconds       [NOMINALLY  1 HOUR]
            self.TEMP_MAX_C =  65           # above this, go to safe mode
            self.TEMP_MIN_C = -10           # below this, go to safe mode
            self.TEMP_HEATER_ON_C  = 10     # below this, turn heater on
            self.TEMP_HEATER_OFF_C = 25     # above this, turn heater off
            self.TEMP_CHG_MAX_C   = 60      # above this, stop charging
            self.TEMP_CHG_UPPER_C = 15      # above this, charge setpoint is -40 mA
            self.TEMP_CHG_LOWER_C = 5       # above this, charge setpoint is -10 mA
            self.TEMP_CHG_MIN_C   = 0       # below this, stop charging
            
            self.CHG_UPPER_SETPT_MA = -45   # nominal setpoint
            self.CHG_LOWER_SETPT_MA = -10   # cold setpoint
            self.CHG_MIN_SETPT_MA   = -8    # minimum setpoint
            
            self.CHG_SETPT_DELTA_MA = 0.5   # if charging current is outside bound, increment potentiometer
            self.CHG_LOW_SETPT_MA = -10    # setpoint for 1/10 C charging
            self.CHG_LOW_SETPT_DELTA_MA = 0.3 #bound for when in low charge mode
            self.DIS_SETPT_MA = 45          # discharge setpoint
            self.DIS_SETPT_DELTA_MA = 0.5
            self.DIS_LOW_SETPT_MA = 4.5     # setpoint for 1/10 C discharging
            self.DIS_LOW_SETPT_DELTA_MA = 0.3 #bound for when in low discharge mode
            
            self.DIS_TRANS_MA = 4.9 #current to transition to low current mode
            self.CHG_VAL_INIT = 9
            self.DIS_VAL_INIT = 120
            self.NUM_CYCLES_PER_TEST = 20    # typically 20
            self.num_boots = 0
            self.ALPHA_EKF = 0.04
            self.ALPHA_CYC = 0.01
            
            self.R_state = 0.015                     # Measurement noise covariance
            self.Q_state = np.diag([1e-6, 1e-3])     # Process noise covariance
            self.Q_param = 1e-1                      # Parameter process noise
            self.R_param = 3.72725e-3                # Parameter measurement noise
        
    def update_parameter(self, file_name, variable, value):
        #If we need to update the params in flight, this should write to the json file
        print('updating params')
        parsed = {} #temporary hold for json info
        try:
            with open(file_name, 'r') as json_in:
                new_msg = json.loads(json_in.read())
                new_msg[variable] = value
                parsed = new_msg
            with open(file_name, 'w') as json_out:
                json_out.write(json.dumps(parsed))
        except Exception:
            print("Issue updating parameter")
            return False
        return True
    
    def fetch_parameter_name(self, index):
        # given index, return variable name
        # for use with update_parameter, for example:
        # PARAMS.update_parameter(params_file, "num_boots", PARAMS.num_boots)
        name_list = ["DT_CHECK_S",         #  0
                "DT_SENSORS_S",            #  1
                "DT_LOG_S",                #  2
                "DT_LOG2_S",               #  3
                "DT_LOG3_S",               #  4
                "DT_HEAT_S",               #  5
                "DT_FAST_S",               #  6
                "DT_SLOW_S",               #  7
                "DT_BACKUP_S",             #  8
                "CHG_LIMIT_V",             #  9
                "DIS_LIMIT_V",             # 10
                "FB_PULSE1_V",             # 11
                "FB_PULSE2_V",             # 12
                "SOC_PULSE1",              # 13
                "SOC_PULSE2",              # 14
                "TIME_CYCLE_REST_S",       # 15
                "TIME_PULSE_REST_S",       # 16
                "TIME_PULSE_TEST_S",       # 17
                "TIME_TEST_REST_S",        # 18
                "TEMP_MAX_C",              # 19
                "TEMP_MIN_C",              # 20
                "TEMP_HEATER_ON_C",        # 21
                "TEMP_HEATER_OFF_C",       # 22
                "TEMP_CHG_MAX_C",          # 23
                "TEMP_CHG_UPPER_C",        # 24
                "TEMP_CHG_LOWER_C",        # 25
                "TEMP_CHG_MIN_C",          # 26
                
                "CHG_UPPER_SETPT_MA",      # 27
                "CHG_LOWER_SETPT_MA",      # 28
                "CHG_MIN_SETPT_MA",        # 29
                
                "CHG_SETPT_DELTA_MA",      # 30
                "CHG_LOW_SETPT_MA",        # 31
                "CHG_LOW_SETPT_DELTA_MA",  # 32
                "DIS_SETPT_MA",            # 33
                "DIS_SETPT_DELTA_MA",      # 34
                "DIS_LOW_SETPT_MA",        # 35
                "DIS_LOW_SETPT_DELTA_MA",  # 36
                
                "DIS_TRANS_MA",            # 37
                "CHG_VAL_INIT",            # 38
                "DIS_VAL_INIT",            # 39
                "NUM_CYCLES_PER_TEST",     # 40
                "num_boots",               # 41
                "ALPHA_EKF",               # 42
                "ALPHA_CYC",               # 43
                
                "R_state",                 # 44
                "Q_state1",                # 45
                "Q_state2",                # 46
                "Q_param",                 # 47
                "R_param"]                 # 48
        if index >= len(name_list) or index < 0:
            return None
        else:
            return name_list[index]
    
    def fetch_parameter_index(self, param_name):
        # this will use the "fetch parameter name" fxn so we only need to maintain one list
        # it seems hacky, but will actually reduce chance we mess something
        for i in range(99):
            if param_name == fetch_parameter_name(i):
                return i
        return -1