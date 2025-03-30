class FSW_PARAMS:
    def __init__(self):
        #TODO - make initialization read from json or similar
        self.DT_LOG_S  = 5              # time step between logs
        self.DT_HEAT_S = 60             # time step between checking heater
        self.DT_FAST_S = 1              # time step between fast loop of EKF
        self.DT_SLOW_S = 1000           # time step between slow loop of EKF
        self.CHG_LIMIT_V = 4.0          # voltage to stop charging
        self.DIS_LIMIT_V = 3.6          # voltage to stop discharging
        self.TIME_TO_REST_S = 30        # time to rest between charge/discharge, seconds
        self.TEMP_MAX_C =  65           # above this, go to safe mode
        self.TEMP_MIN_C = -10           # below this, go to safe mode
        self.TEMP_HEATER_ON_C  = 10     # below this, turn heater on
        self.TEMP_HEATER_OFF_C = 25     # above this, turn heater off
        self.TEMP_CHG_MAX_C   = 60      # above this, stop charging
        self.TEMP_CHG_UPPER_C = 15      # above this, charge setpoint is -40 mA
        self.TEMP_CHG_LOWER_C = 5       # above this, charge setpoint is -10 mA
        self.TEMP_CHG_MIN_C   = 0       # below this, stop charging
        self.CHG_UPPER_SETPT_MA = -40   # nominal setpoint
        self.CHG_LOWER_SETPT_MA = -10   # cold setpoint
        self.CHG_MIN_SETPT_MA   = -8    # minimum setpoint
        self.CHG_SETPT_DELTA_MA = 0.5   # if charging current is outside bound, increment potentiometer
        self.DIS_SETPT_MA = 40          # discharge setpoint
        self.DIS_SETPT_DELTA_MA = 0.5
        self.DIS_TRANS_MA = 10
        self.CHG_VAL_INIT = 9
        self.DIS_VAL_INIT = 120
        self.R_SHUNT_OHMS = [1.0, 1.0, 1.0]
        
    def update_params():
        #TODO if we need to update the params in flight, this should write to the json file
        print('updating params')
