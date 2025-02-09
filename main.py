import sys
import smbus2
import serial
import time
import csv
import RPi.GPIO as GPIO

# =============================
#  CONFIGURATION SETTINGS
# =============================
# I2C addresses
INA236_ADDRS = []  # Current & Voltage Sensors
TMP112_ADDRS = []  # Temperature Sensors
MCP4452_ADDRS = []  # Digital Potentiometers

# UART Configuration
UART_PORT = "/dev/serial0"
BAUDRATE = 9600
TIMEOUT = 1

# I2C Bus initialization
I2C_BUS = smbus2.SMBus(1)

# GPIO Settings for controlling charging and discharging
CHARGE_CONTROL_PIN = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(CHARGE_CONTROL_PIN, GPIO.OUT, initial=GPIO.LOW)

# =========================
#  FUNCTION DEFINITIONS
# =========================
def read_ina236(address):
    """Reads voltage and current from INA236 sensor."""
    try:
        voltage = I2C_BUS.read_word_data(address, 0x02) * 1.25 / 1000
        current = I2C_BUS.read_word_data(address, 0x04) * 1.25 / 1000
        return voltage, current
    except Exception as e:
        print(f"Error reading INA236 at {hex(address)}: {e}")
        return None, None

def read_temperature(address):
    """Reads temperature from TMP112 sensor."""
    try:
        temp = I2C_BUS.read_word_data(address, 0x00) / 256.0
        return temp
    except Exception as e:
        print(f"Error reading TMP112 at {hex(address)}: {e}")
        return None

def adjust_potentiometer(address, level):
    """Adjusts the resistance of a potentiometer."""
    try:
        I2C_BUS.write_byte_data(address, 0x00, level)
    except Exception as e:
        print(f"Error adjusting MCP4452 at {hex(address)}: {e}")

def control_charging(enable):
    """Enables or disables battery charging."""
    GPIO.output(CHARGE_CONTROL_PIN, GPIO.HIGH if enable else GPIO.LOW)

def log_battery_data(voltage, current, temperature, charge_state):
    with open("battery_log.csv", "a", newline='') as file:
        csv.writer(file).writerow([time.time(), voltage, current, temperature, charge_state])

def battery_cycle_manager():
    while True:
        for address in INA236_ADDRS:
            voltage, current = read_ina236(address)
            temperature = read_temperature(TMP112_ADDRS[INA236_ADDRS.index(address)])
            if None in (voltage, current, temperature):
                continue
            charge_state = voltage < 3.5
            control_charging(charge_state)
            adjust_potentiometer(MCP4452_ADDRS[INA236_ADDRS.index(address)], 200 if charge_state else 50)
            log_battery_data(voltage, current, temperature, charge_state)
        time.sleep(1)

def send_telemetry(voltage, current, temperature):
    try:
        with serial.Serial(UART_PORT, baudrate=BAUDRATE, timeout=TIMEOUT) as uart:
            uart.write(f"BATTERY STATUS: {voltage:.2f}V, {current:.2f}A, {temperature:.1f}C\n".encode())
    except Exception as e:
        print(f"UART Error: {e}")

# ===========
#  MAIN 
# ===========
if __name__ == "__main__":
    try:
        battery_cycle_manager()
    except KeyboardInterrupt:
        GPIO.cleanup()
