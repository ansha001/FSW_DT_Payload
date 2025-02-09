import sys
import os
import serial
import time
import csv
import logging

# ==============================
#  MOCK I2C AND GPIO FOR WINDOWS
# =============================

if sys.platform == "win32":
    class SMBusMock:
        """Mock I2C Bus for Windows testing."""
        def write_byte_data(self, addr, reg, value):
            print(f"Mock I2C Write - Addr: {hex(addr)}, Reg: {hex(reg)}, Value: {value}")

        def read_word_data(self, addr, reg):
            print(f"Mock I2C Read - Addr: {hex(addr)}, Reg: {hex(reg)}")
            return 500  # mock sensor value
    
    class GPIOMock:
        """Mock GPIO for Windows testing."""
        BCM = "BCM"
        OUT = "OUT"
        LOW = "LOW"
        HIGH = "HIGH"

        def setmode(self, mode):
            print("Mock GPIO setmode:", mode)

        def setup(self, pin, mode, initial=None):
            print(f"Mock GPIO setup - Pin: {pin}, Mode: {mode}, Initial: {initial}")

        def output(self, pin, value):
            print(f"Mock GPIO output - Pin: {pin}, Value: {value}")

        def cleanup(self):
            print("Mock GPIO cleanup")

    I2C_BUS = SMBusMock()
    GPIO = GPIOMock()

else:
    import smbus2 
    import RPi.GPIO as GPIO
    I2C_BUS = smbus2.SMBus(1)

# =============================
#  CONFIGURATION SETTINGS
# =============================

# I2C addresses
INA236_ADDRS = [0x40]  
TMP112_ADDRS = [0x48] 
MCP4452_ADDRS = [0x2E] 

# UART Configuration
UART_PORT = "COM3" if sys.platform == "win32" else "/dev/serial0"
BAUDRATE = 9600
TIMEOUT = 1

# GPIO Settings for controlling charging and discharging
CHARGE_CONTROL_PIN = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(CHARGE_CONTROL_PIN, GPIO.OUT, initial=GPIO.LOW)

from datetime import datetime
LOG_FILE_TXT = "battery_log.txt"
LOG_FILE_CSV = "battery_log.csv"

logging.basicConfig(
    filename=LOG_FILE_TXT,
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
    encoding="utf-8",
)

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

def adjust_potentiometer(address, level, mode="charging"):
    """Adjusts the resistance of a potentiometer."""
    try:
        if mode == "charging":
            level = max(0, min(level, 400))  
        else:
            level = max(0, min(level, 100))  
        
        I2C_BUS.write_byte_data(address, 0x00, level)
    except Exception as e:
        print(f"Error adjusting MCP4452 at {hex(address)}: {e}")

def log_battery_data(voltage, current, temperature, charge_state):
    """Logs battery state to a CSV file and text log."""
    
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    log_entry = f"Voltage: {voltage:.2f}V, Current: {current:.3f}A, Temp: {temperature:.1f}°C, Charging: {'ON' if charge_state else 'OFF'}"

    logging.info(log_entry)

    file_exists = os.path.exists(LOG_FILE_CSV)
    write_headers = not file_exists or os.stat(LOG_FILE_CSV).st_size == 0 

    with open(LOG_FILE_CSV, "a", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)

        if write_headers:
            writer.writerow(["Timestamp", "Voltage (V)", "Current (A)", "Temperature (C)", "Charging State"])

        writer.writerow([timestamp, voltage, current, temperature, "ON" if charge_state else "OFF"])


def battery_cycle_manager():
    while True:
        for address in INA236_ADDRS:
            voltage, current = read_ina236(address)
            temperature = read_temperature(TMP112_ADDRS[INA236_ADDRS.index(address)])

            if None in (voltage, current, temperature):
                continue

            if voltage < 3.5:
                target_current = 45  
                control_charging(target_current, current)
            else:
                discharge_mode = "low" if voltage < 3.6 else "high"
                control_discharging(discharge_mode)

            log_battery_data(voltage, current, temperature, voltage < 3.5)

        time.sleep(1)

def send_telemetry(voltage, current, temperature, discharge_mode):
    try:
        with serial.Serial(UART_PORT, baudrate=BAUDRATE, timeout=TIMEOUT) as uart:
            uart.write(f"BATTERY: {voltage:.2f}V, {current:.2f}A, {temperature:.1f}C, Mode: {discharge_mode}\n".encode())
    except Exception as e:
        print(f"UART Error: {e}")

def control_charging(target_current, current_reading):
    """Uses controller to regulate charging current."""
    Kp = 0.1  # Proportional gain (Tunable)
    Ki = 0.01  # Integral gain (Tunable)
    integral = 0

    error = target_current - current_reading
    integral += error

    adjustment = Kp * error + Ki * integral
    new_pot_level = int(max(0, min(400, 200 + adjustment))) 
    
    adjust_potentiometer(MCP4452_ADDRS[0], new_pot_level, mode="charging")
    GPIO.output(CHARGE_CONTROL_PIN, GPIO.HIGH if target_current > current_reading else GPIO.LOW)

def control_discharging(mode):
    """Controls discharge current via potentiometer and MOSFET."""
    if mode == "low":
        pot_value = 100 
    else:
        pot_value = 0 
    
    adjust_potentiometer(MCP4452_ADDRS[1], pot_value, mode="discharging")


# =========== MAIN ===========
if __name__ == "__main__":
    try:
        battery_cycle_manager()
    except KeyboardInterrupt:
        GPIO.cleanup()
