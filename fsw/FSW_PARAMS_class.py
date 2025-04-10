class FSW_PARAMS:
    def __init__(self):
        #TODO - make initialization read from json or similar
        self.DT_CHECK_S = 0.08          # time step between important checks
        self.DT_SENSORS_S = 0.1         # time step between reading sensors
        self.DT_LOG_S  = 1.1              # time step between logs
        self.DT_HEAT_S = 160             # time step between checking heater
        self.DT_FAST_S = 1              # time step between fast loop of EKF
        self.DT_SLOW_S = 1000           # time step between slow loop of EKF
        self.CHG_LIMIT_V = 4.20         # voltage to stop charging
        self.DIS_LIMIT_V = 2.90         # voltage to stop discharging
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
        self.R_SHUNT_OHMS = [1.0, 1.0, 1.0]
        self.NUM_CYCLES_PER_TEST = 20    # typically 20
        
    def update_params():
        #TODO if we need to update the params in flight, this should write to the json file
        print('updating params')
