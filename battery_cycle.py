import RPi.GPIO as GPIO
import time
import smbus2
import os

GPIO.setmode(GPIO.BOARD) # Physical pin numbering using BOARD (set GPIO.BCM to use broadcom GPIO numbering)

CHG_PINS = [16, 18, 22]  # GPIO 23, 24, 25
DIS_PINS = [29, 31, 33]  # GPIO 5, 6, 13
CUR_PINS = [32, 36, 37]  # GPIO 12, 16, 26

# i2c addresses for potentiometers
POT_ADDRS = {
    1: 0x2C,
    2: 0x2E,
    3: 0x2D,
    4: 0x2F
}

WIPER_REGS = [0x00, 0x01, 0x06, 0x07] # MCP4452 Wiper registers

MAX_VOLTAGE = 4.2
MIN_VOLTAGE = 3.0
MAX_TEMP = 45  # dummy value (Celsius)
MIN_TEMP = 0   # dummy value

bus = smbus2.SMBus(1)

def set_pot_wiper(i2c_address, wiper, value, mode="block"):
    if value < 0 or value > 256:
        raise ValueError("Wiper value out of range (0-256)")
    
    wiper_register = WIPER_REGS[wiper]
    command_byte = (wiper_register << 4) | ((value >> 8) & 0x01)  # Include D8 bit
    lsb = value & 0xFF # lower 8 bits

    try:
        if mode == "block":
            bus.write_i2c_block_data(i2c_address, command_byte, [lsb])
        else:
            bus.write_byte_data(i2c_address, wiper_register, lsb) ## (can be omitted if decide to proceed with block write)
    except OSError as e:
        print(f"Error setting wiper {wiper} on Potentiometer at address {i2c_address}: {e}")

# sensor functions (to be completed)
def read_battery_voltage(sensor_id):
    return 3.7  # placeholdetr voltage

def read_battery_current(sensor_id):
    return 0.0  # placeholder current

def read_temperature(sensor_id):
    return 20  # placeholder temp

# func for delay (and not use sleep)
def wait_for_interval(last_time, interval):
    current_time = time.monotonic()
    if current_time - last_time >= interval:
        return True, current_time
    return False, last_time

###  Start-up sequence ###
# set up GPIO pins as output
for pin_group in [CHG_PINS, DIS_PINS, CUR_PINS]:
    for pin in pin_group:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)  #(open switches)

# set POT[1,2,3] to max for all wipers, POT4 to min
for pot in [1, 2, 3]:
    for wiper in range(4):
        set_pot_wiper(POT_ADDRS[pot], wiper, 256, mode="block")
for wiper in range(4):
    set_pot_wiper(POT_ADDRS[4], wiper, 0, mode="block")


### cycle for battery 1
cycle = 0
for cycle in range(20):

    ### Charging ####

    print(f"Cycle {cycle + 1}/20: Charging Phase")
    VAL = 9 #(as per notes)
    set_pot_wiper(POT_ADDRS[1], 0, 2, mode="block")
    set_pot_wiper(POT_ADDRS[1], 1, 2, mode="block")
    set_pot_wiper(POT_ADDRS[1], 2, 2, mode="block")
    set_pot_wiper(POT_ADDRS[1], 3, 1, mode="block")

    # Set CHG ON
    for pin in CHG_PINS:
        GPIO.output(pin, GPIO.HIGH)

    last_time = time.monotonic()
    while read_battery_voltage(1) < MAX_VOLTAGE:
        ready, last_time = wait_for_interval(last_time, 0.1)  # 100 ms delay
        if ready: 
            temp = read_temperature(1)
            if temp > 15:
                charging_setpoint = -40 #mA
            elif temp > 5:
                charging_setpoint = -3 * temp + 5 
            else:
                charging_setpoint = -10

            battery_current = read_battery_current(1)
            if battery_current < (charging_setpoint - 3):
                VAL = min(VAL + 1, 1024)
            elif battery_current > (charging_setpoint + 3):
                VAL = max(VAL - 1, 0)

            # set wipers based on VAL
            set_pot_wiper(POT_ADDRS[1], 3, VAL // 4, mode="block") # floor division
            set_pot_wiper(POT_ADDRS[1], 2, VAL // 4 + ((VAL % 4) > 3), mode="block")
            set_pot_wiper(POT_ADDRS[1], 1, VAL // 4 + ((VAL % 4) > 2), mode="block")
            set_pot_wiper(POT_ADDRS[1], 0, VAL // 4 + ((VAL % 4) > 1), mode="block")

            if temp > MAX_TEMP or temp < MIN_TEMP:
                print("Temperature out of range. Stopping experiment..")

                # Safe mode
                for pin in CHG_PINS: #set CHG OFF
                    GPIO.output(pin, GPIO.LOW)  
                for wiper in range(4):
                    set_pot_wiper(POT_ADDRS[1], wiper, 256, mode="block")
                break


# set CHG OFF (before discharging)
for pin in CHG_PINS: 
    GPIO.output(pin, GPIO.LOW)  

    ### Discharging ###

    print(f"Cycle {cycle + 1}/20: Discharging Phase")
    VAL0 = 76 #(as per notes)
    set_pot_wiper(POT_ADDRS[4], 0, VAL0, mode="block")

    # set CUR and DIS pins to HIGH
    for pin in CUR_PINS:
        GPIO.output(pin, GPIO.HIGH)
    for pin in DIS_PINS:
        GPIO.output(pin, GPIO.HIGH)

    last_time = time.monotonic()
    while read_battery_voltage(1) > MIN_VOLTAGE:
        ready, last_time = wait_for_interval(last_time, 0.1)  # 100 ms delay
        if ready:
            battery_current = read_battery_current(1)
            if battery_current < 37:
                VAL0 += 1
                set_pot_wiper(POT_ADDRS[4], 0, VAL0, mode="block")
            elif battery_current > 43:
                VAL0 -= 1
                set_pot_wiper(POT_ADDRS[4], 0, VAL0, mode="block")

    # Set DIS OFF
    for pin in DIS_PINS:
        GPIO.output(pin, GPIO.LOW)
    for pin in CUR_PINS:
        GPIO.output(pin, GPIO.LOW)

# Cleanup
GPIO.cleanup()
bus.close()
print("Experiment Complete")
