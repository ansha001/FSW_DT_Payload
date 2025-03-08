import smbus2
import subprocess
from config import bus, TMP_ADDRS, INA_ADDRS, SERIAL_PORT, BAUDRATE, SYS_TIMEOUT
import serial

class SensorReader:
    def __init__(self):
        self.serial_conn = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=SYS_TIMEOUT)
        self.initialize_system()

    def initialize_system(self):
        self.serial_conn.write('SYST:REM\n'.encode()) 
        self.serial_conn.write('Syst:beep\n'.encode())  
        self.serial_conn.write('OUTP ON\n'.encode())

    def read_temperature(self, addr):
        try:
            # Read two bytes
            raw_data = bus.read_i2c_block_data(addr, 0x00, 2)

            # Combine bytes into a single integer (big-endian format)
            raw_temp = (raw_data[0] << 4) | (raw_data[1] >> 4)

            # handle negative temperatures 
            if raw_temp & (1 << 11):  # check if sign bit is set
                raw_temp -= 1 << 12  # apply two's complement for negative values
            return raw_temp * 0.0625
        
        except Exception as e:
            print(f"Error reading temperature: {e}")
            return None

    def read_voltage(self, addr):
        try:
            raw_data = bus.read_word_data(addr, 0x02)
            raw_data = ((raw_data << 8) & 0xFF00) | (raw_data >> 8)
            return 1.28 * raw_data * 1.25 / 1000  
        except Exception as e:
            print(f"Error reading voltage: {e}")
            return None

    def read_current(self, addr, state="CHG"):
        try:
            if state == "DIS":
                self.serial_conn.write('MEAS:CURR?\n'.encode())
                current_from_ps = float(self.serial_conn.readline().decode().strip())
                return abs(current_from_ps) * 1000 # convert tomA
            else:
                shunt_voltage_raw = bus.read_word_data(addr, 0x01)
                shunt_voltage_raw = ((shunt_voltage_raw << 8) & 0xFF00) | (shunt_voltage_raw >> 8)
                # handle negative values
                if shunt_voltage_raw & (1 << 15):
                    shunt_voltage_raw -= (1 << 16)

                return (shunt_voltage_raw * 2.5) / 1  # Assuming R_shunt = 1 Ohm
            
        except Exception as e:
            print(f"Error reading current: {e}")
            return None


    def get_cpu_temperature(self):
        output = subprocess.check_output(['vcgencmd', 'measure_temp'])
        return float(output[5:9])

    def get_cpu_voltage(self):
        output = subprocess.check_output(['vcgencmd', 'measure_volts'])
        return float(output[5:11])
    
    def ping_sensors(self, cell_state):
        #read all the sensors
        temp0 = self.read_temperature(TMP_ADDRS[0])
        temp1 = self.read_temperature(TMP_ADDRS[1])
        temp2 = 0  #skip until rev 2 of board
        temp_CPU = self.get_CPU_temperature()
        volt0 = self.read_voltage(INA_ADDRS[0])
        volt1 = self.read_voltage(INA_ADDRS[1])
        volt2 = 0 # skip until rev 2 of board
        volt_CPU = self.get_CPU_voltage()
        curr0 = self.read_current(INA_ADDRS[0], cell_state)
        curr1 = self.read_current(INA_ADDRS[1], cell_state)
        curr2 = 0 #skip until rev 2 of board
        sensor_data = [temp0,temp1,temp2,temp_CPU,volt0,volt1,volt2,volt_CPU,curr0,curr1,curr2]
        return sensor_data
