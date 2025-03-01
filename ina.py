import smbus2
import time
import RPi.GPIO as GPIO
import serial

bus = smbus2.SMBus(1)

INA1 = 0x40  

EN_CHG_GPIO_LIST = [23,24,25] #GPIO 23,24,25 is physical pins 16,18,22
EN_DIS_GPIO_LIST = [ 5, 6,13] #GPIO  5, 6,13 is physical pins 29,31,33
EN_CUR_GPIO_LIST = [12,16,26] #GPIO 12,16,26 is physical pins 32,36,37

#SERIAL VARS
SERIAL_PORT = '/dev/ttyUSB0'   #'/dev/serial0'  # '/dev/ttyAMA0'
BAUDRATE = 9600
SYS_TIMEOUT = 0.5 #seconds
ser          = serial.Serial()
ser.port     = SERIAL_PORT
ser.baudrate = BAUDRATE
ser.parity   = serial.PARITY_NONE
ser.timeout  = SYS_TIMEOUT
ser.stopbits = serial.STOPBITS_ONE
ser.bytesize = serial.EIGHTBITS

def read_ina236a_data():
    try:
        # Read two bytes from each register (big-endian format)
        bus_voltage_raw   = bus.read_word_data(INA1, 0x02)
        shunt_voltage_raw = bus.read_word_data(INA1, 0x01)
        
        # Read two bytes from shunt register
        raw_data = bus.read_i2c_block_data(INA1, 0x01, 2)
        # Combine bytes into a single integer (big-endian format)
        raw_temp = (raw_data[0] << 4) | (raw_data[1] >> 4)
        # negative temperatures 
        if raw_temp & (1 << 15):  # check if sign bit is set
            raw_temp -= 1 << 16  # apply two's complement for negative values
        print(bin(raw_data[0]))
        print(bin(raw_data[1]))
        
        current_raw = bus.read_word_data(INA1, 0x04)
        power_raw = bus.read_word_data(INA1, 0x03)
        #print(bin(shunt_voltage_raw))
        
        # Swap bytes since INA236A sends data in little-endian format
        bus_voltage_raw   = ((bus_voltage_raw << 8) & 0xFF00) | (bus_voltage_raw >> 8)
        #shunt_voltage_raw = ((shunt_voltage_raw << 8) & 0xFF00) | (shunt_voltage_raw >> 8)
        #current_raw = ((current_raw << 8) & 0xFF00) | (current_raw >> 8)
        #power_raw = ((power_raw << 8) & 0xFF00) | (power_raw >> 8)
        
        # Convert raw values based on datasheet conversion factors
        bus_voltage = 1.28 * bus_voltage_raw * 1.25 / 1000  # in Volts, and apply calibration
        shunt_voltage = raw_temp * 2.5 / 1000  # in mV
        current = shunt_voltage * 1.0  # in mA Example factor (should be calibrated)
        power = current * bus_voltage  # Example factor (should be calibrated)
        
        ser.write('MEAS:CURR?\n'.encode())
        I_meas = ser.readline().decode()
        
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
    GPIO.setup(EN_CHG_GPIO_LIST[0], GPIO.OUT)
    GPIO.output(EN_DIS_GPIO_LIST[0], GPIO.HIGH)
    GPIO.output(EN_CUR_GPIO_LIST[0], GPIO.LOW)
    GPIO.output(EN_CHG_GPIO_LIST[0], GPIO.LOW)
    
    ser.open()
    ser.write('Syst:beep\n'.encode())
    ser.write('OUTP ON\n'.encode())
    print('power supply on')
    
    while True:
        ina_data = read_ina236a_data()
        if ina_data:
            print("INA236A Sensor Data:")
            for key, value in ina_data.items():
                print(f"  {key}: {value:.3f}")
        
        time.sleep(4) 
