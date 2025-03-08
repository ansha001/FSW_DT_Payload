import smbus2

# I2C addresses
INA_ADDRS = [0x40, 0x41, 0x43]  # INA 0,1,2
TMP_ADDRS = [0x48, 0x49, 0x4B]  # TMP 0,1,2
POT_ADDRS = [0x2C, 0x2E, 0x2D, 0x2F]  # POT 0,1,2,3
POT_REGS  = [0x00, 0x01, 0x06, 0x07]

# GPIO Pins
EN_CHG_GPIO_LIST = [23, 24, 25] #GPIO 23,24,25 is physical pins 16,18,22
EN_DIS_GPIO_LIST = [5, 6, 13] #GPIO  5, 6,13 is physical pins 29,31,33
EN_CUR_GPIO_LIST = [12, 16, 26] #GPIO 12,16,26 is physical pins 32,36,37

# Logging Parameters
LOG_FREQ = 0.50 #log at X Hz
DT_LOG = 1 / LOG_FREQ #time step between logs

# Voltage Limits
V_CHG_LIMIT = 4.0
V_DIS_LIMIT = 3.2

# Temperature Limits
TEMP_MAX = 60
TEMP_MIN = -10
TEMP_CHG_UPPER = 15
TEMP_CHG_LOWER = 5

# Setpoint and delta values
CHG_UPPER_SETPT = -40
CHG_LOWER_SETPT = -10
CHG_SETPT_DELTA = 2
DIS_SETPT = 40
DIS_SETPT_DELTA = 2

# Serial Communication
SERIAL_PORT = '/dev/ttyUSB0'
BAUDRATE = 9600
SYS_TIMEOUT = 0.5


# Initialize I2C Bus
bus = smbus2.SMBus(1)
