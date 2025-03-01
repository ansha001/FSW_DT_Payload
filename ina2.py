import smbus2
import time
import RPi.GPIO as GPIO

bus = smbus2.SMBus(1)

INA1 = 0x40  

EN_CHG_GPIO_LIST = [23,24,25] #GPIO 23,24,25 is physical pins 16,18,22
EN_DIS_GPIO_LIST = [ 5, 6,13] #GPIO  5, 6,13 is physical pins 29,31,33
EN_CUR_GPIO_LIST = [12,16,26] #GPIO 12,16,26 is physical pins 32,36,37

def read_ina236a_data():
    try:
        # Read two bytes from each register (big-endian format)
        bus_voltage_raw   = bus.read_word_data(INA1, 0x02)
        shunt_voltage_raw = bus.read_word_data(INA1, 0x01)
        current_raw = bus.read_word_data(INA1, 0x04)
        power_raw = bus.read_word_data(INA1, 0x03)
        
        # Swap bytes since INA236A sends data in little-endian format
        bus_voltage_raw   = ((bus_voltage_raw << 8) & 0xFF00) | (bus_voltage_raw >> 8)
        shunt_voltage_raw = ((shunt_voltage_raw << 8) & 0xFF00) | (shunt_voltage_raw >> 8)
        current_raw = ((current_raw << 8) & 0xFF00) | (current_raw >> 8)
        power_raw = ((power_raw << 8) & 0xFF00) | (power_raw >> 8)
        print(shunt_voltage_raw)
        # Convert raw values based on datasheet conversion factors
        bus_voltage = 1.28 * bus_voltage_raw * 1.25 / 1000  # in Volts, and apply calibration
        shunt_voltage = shunt_voltage_raw * 2.5 / 1000  # in mV
        current = current_raw * 1.25  # Example factor (should be calibrated)
        power = power_raw * 25  # Example factor (should be calibrated)
        
        return {
            'Bus voltage (V)': bus_voltage,
            'Shunt voltage (mV)': shunt_voltage,
           # 'Current (mA)': current,
           # 'Power (mW)': power
        }
    except Exception as e:
        print(f"Error reading INA sensor: {e}")
        return None

if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(EN_DIS_GPIO_LIST[0], GPIO.OUT)
    GPIO.setup(EN_CUR_GPIO_LIST[0], GPIO.OUT)
    GPIO.output(EN_DIS_GPIO_LIST[0], GPIO.HIGH)
    GPIO.output(EN_CUR_GPIO_LIST[0], GPIO.HIGH)
    while True:
        ina_data = read_ina236a_data()
        if ina_data:
            print("INA236A Sensor Data:")
            for key, value in ina_data.items():
                print(f"  {key}: {value:.2f}")
        
        time.sleep(5) 
