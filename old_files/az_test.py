import sys
import os
import serial
import time
import csv
import logging
import smbus2
import RPi.GPIO as GPIO
import subprocess

# I2C addresses
INA_ADDRS = [0x40, 0x41, 0x43] #INA 0,1,2
TMP_ADDRS = [0x48, 0x49, 0x4B] #TMP 0,1,2
POT_ADDRS = [0x2C, 0x2E, 0x2D, 0x2F] #POT 0,1,2,3
POT_REGS  = [0x00, 0x01, 0x06, 0x07]

# GPIO Pins
EN_CHG_GPIO_LIST = [23, 24, 25] #GPIO 23,24,25 is physical pins 16,18,22
EN_DIS_GPIO_LIST = [5, 6, 13] #GPIO  5, 6,13 is physical pins 29,31,33
EN_CUR_GPIO_LIST = [12, 16, 26] #GPIO 12,16,26 is physical pins 32,36,37

#Init I2C Bus
bus = smbus2.SMBus(1)

#set test parameters
LOG_FREQ = 0.50 #log at X Hz
DT_LOG = 1/LOG_FREQ #time step between logs
V_CHG_LIMIT = 4.0
V_DIS_LIMIT = 3.2
# set other parameters TODO read this from a json file or similar
TEMP_MAX =  60  #above this, go to safe mode
TEMP_MIN = -10  #below this, go to safe mode
TEMP_CHG_UPPER = 15
TEMP_CHG_LOWER = 5
CHG_UPPER_SETPT = -40
CHG_LOWER_SETPT = -10
CHG_SETPT_DELTA = 2
DIS_SETPT = 40
DIS_SETPT_DELTA = 2

SERIAL_PORT = '/dev/ttyUSB0'
BAUDRATE = 9600
SYS_TIMEOUT = 0.5
ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=SYS_TIMEOUT)

file_name = 'ZZ_log_'
trial = 1
while os.path.exists(file_name + str(trial)+ '.csv'):
    trial += 1
file_name = file_name + str(trial) + '.csv'
header = ['Time (s)','Temp0','Temp1','Temp2','Temp_CPU','Volt0','Volt1','Volt2','Volt_CPU','Curr0','Curr1','Curr2']
with open(file_name, 'w', encoding='UTF8', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(header)
    
chg_vals = [9, 9, 9]
dis_vals = [76, 76, 76]

def init_gpio():
    GPIO.setmode(GPIO.BCM)
    for pin_group in [EN_CHG_GPIO_LIST, EN_DIS_GPIO_LIST, EN_CUR_GPIO_LIST]:
        for pin in pin_group:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
    
def set_wiper(i2c_address, wiper_register, value):
    #set_wiper(POT1_ADDR, WIPER_0, value, mode="block")
    print(f"Setting wiper value in {wiper_register} to {value}")
    if value < 0 or value > 256:
        raise ValueError("Wiper value out of range (0-256)")
    command_byte = (wiper_register << 4) | ((value >> 8) & 0x01)  
    lsb = value & 0xFF  

    try:
        bus.write_i2c_block_data(i2c_address, command_byte, [lsb])
        
    except OSError as e:
        print(f"Error setting wiper {wiper_register} on {i2c_address}: {e}")
        return -1

def set_POT(cell_num, VAL):
    # given "10-bit" potentiometer value
    # set each wiper in the potentiometer to the corresponding value
    VAL = max(0, min(1024, VAL))
    A = int(VAL/4)
    B = A + (VAL%4 > 2)
    C = A + (VAL%4 > 1)
    D = A + (VAL%4 > 0)
    set_wiper(POT_ADDRS[cell_num], POT_REGS[0], A)
    set_wiper(POT_ADDRS[cell_num], POT_REGS[1], B)
    set_wiper(POT_ADDRS[cell_num], POT_REGS[2], C)
    set_wiper(POT_ADDRS[cell_num], POT_REGS[3], D)
    
def read_wiper(i2c_address, wiper_register, mode="block"):
    try:
        if mode == "byte":
            data = bus.read_byte_data(i2c_address, wiper_register)
            return data
        elif mode == "block":
            command_byte = (wiper_register << 4) | (0x03 << 2)  # read command 11
#            bus.write_byte(i2c_address, command_byte)
            data = bus.read_i2c_block_data(i2c_address, command_byte, 2)
            value = ((data[0] & 0x01) << 8) | data[1]  # combine MSB and LSB for 9bit value
            return value
        else:
            raise ValueError("invalid mode")
    except OSError as e:
        print(f"Error reading wiper {wiper_register}: {e}")
        return -1
    
def read_temperature(TMP_ADDR = 0x48):
    try:
        # Read two bytes 
        raw_data = bus.read_i2c_block_data(TMP_ADDR, 0x00, 2)
        
        # Combine bytes into a single integer (big-endian format)
        raw_temp = (raw_data[0] << 4) | (raw_data[1] >> 4)
        
        # negative temperatures 
        if raw_temp & (1 << 11):  # check if sign bit is set
            raw_temp -= 1 << 12  # apply two's complement for negative values
        
        temp_c = raw_temp * 0.0625
        return temp_c
    except Exception as e:
        print(f"Error : {e}")
        return None

def get_CPU_temperature():
    output = subprocess.check_output(['vcgencmd','measure_temp'])
    temperature = float(output[5:9])
    #print('CPU Temp is ',temperature,'*C')
    return temperature

def get_CPU_voltage():
    output = subprocess.check_output(['vcgencmd','measure_volts'])
    voltage = float(output[5:11])
    #print('CPU Voltage is ',voltage,'V')
    return voltage

def read_voltage(INA_ADR = 0x40):
    #bus_voltage = 7 #TODO implement voltage read from INA
    bus_voltage_raw = bus.read_word_data(INA_ADR, 0x02)  #should return 16-bit only positive values
    bus_voltage_raw = ((bus_voltage_raw << 8) & 0xFF00) | (bus_voltage_raw >> 8)
    bus_voltage = 1.28 * bus_voltage_raw * 1.25 / 1000  # in Volts
    return bus_voltage

def read_current(INA_ADR = 0x40, state="CHG"):
    if state=="DIS":
        try:
            #ser.open()
            ser.write('MEAS:CURR?\n'.encode())
            I_ps = ser.readline().decode().strip()
            #ser.close()
            print(I_ps)
            current_from_ps = float(I_ps)

            return abs(current_from_ps)*1000

        except Exception as e:
            print(f"Error reading power supply: {e}")
            return None 
    else:
        #current = 7 #TODO implment current read from INA
        R_shunt = 1 #Ohm
        shunt_voltage_raw = bus.read_word_data(INA_ADR, 0x01)  #should return 16-bit two's complement
        shunt_voltage_raw = ((shunt_voltage_raw << 8) & 0xFF00) | (shunt_voltage_raw >> 8)

        # apply two’s complement correction if negative
        if shunt_voltage_raw & (1 << 15): # & with 100000000000000 to check if msb is 1 or 0
            shunt_voltage_raw -= (1 << 16)

        shunt_voltage = shunt_voltage_raw * 2.5  # in V
        current = shunt_voltage / R_shunt
    return current
        
def ping_sensors(state):
    #read all the sensors
    temp0 = read_temperature(TMP_ADDRS[0])
    temp1 = read_temperature(TMP_ADDRS[1])
    temp2 = 0  #skip until rev 2 of board
    temp_CPU = get_CPU_temperature()
    volt0 = read_voltage(INA_ADDRS[0])
    volt1 = read_voltage(INA_ADDRS[1])
    volt2 = 0 # skip until rev 2 of board
    volt_CPU = get_CPU_voltage()
    curr0 = read_current(INA_ADDRS[0], state)
    curr1 = read_current(INA_ADDRS[1], state)
    curr2 = 0 #skip until rev 2 of board
    sensor_data = [temp0,temp1,temp2,temp_CPU,volt0,volt1,volt2,volt_CPU,curr0,curr1,curr2]
    return sensor_data

def log_sensor_data(time, data):
    with open(file_name, 'a', encoding='UTF8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([str(time),str(data[0]),str(data[1]),str(data[2]),str(data[3]),str(data[4]),str(data[5]),str(data[6]),str(data[7]),str(data[8]),str(data[9]),str(data[10])])
    

def set_GPIO(cell_num, state, GPIO_LIST):
    # manipulate EN_CHG GPIO pins according to cell num and desired state
    if state=='ON' or state=='HIGH':
        GPIO.output(GPIO_LIST[cell_num], GPIO.HIGH) #set HIGH
    else:
        GPIO.output(GPIO_LIST[cell_num], GPIO.LOW) #set LOW
        
def safe_board(cell = -1):
    if cell == -1:
        #set everything to safe settings
        set_GPIO(0, 'OFF', EN_CHG_GPIO_LIST)
        set_GPIO(1, 'OFF', EN_CHG_GPIO_LIST)
        set_GPIO(2, 'OFF', EN_CHG_GPIO_LIST)
        set_GPIO(0, 'OFF', EN_DIS_GPIO_LIST)
        set_GPIO(1, 'OFF', EN_DIS_GPIO_LIST)
        set_GPIO(2, 'OFF', EN_DIS_GPIO_LIST)
        set_GPIO(0, 'LOW', EN_CUR_GPIO_LIST)
        set_GPIO(1, 'LOW', EN_CUR_GPIO_LIST)
        set_GPIO(2, 'LOW', EN_CUR_GPIO_LIST)
        set_wiper(POT_ADDRS[0], POT_REGS[0], 256)
        set_wiper(POT_ADDRS[0], POT_REGS[1], 256)
        set_wiper(POT_ADDRS[0], POT_REGS[2], 256)
        set_wiper(POT_ADDRS[0], POT_REGS[3], 256)
        set_wiper(POT_ADDRS[1], POT_REGS[0], 256)
        set_wiper(POT_ADDRS[1], POT_REGS[1], 256)
        set_wiper(POT_ADDRS[1], POT_REGS[2], 256)
        set_wiper(POT_ADDRS[1], POT_REGS[3], 256)
        set_wiper(POT_ADDRS[2], POT_REGS[0], 256)
        set_wiper(POT_ADDRS[2], POT_REGS[1], 256)
        set_wiper(POT_ADDRS[2], POT_REGS[2], 256)
        set_wiper(POT_ADDRS[2], POT_REGS[3], 256)
        set_wiper(POT_ADDRS[3], POT_REGS[0], 0)
        set_wiper(POT_ADDRS[3], POT_REGS[1], 0)
        set_wiper(POT_ADDRS[3], POT_REGS[2], 0)
        set_wiper(POT_ADDRS[3], POT_REGS[3], 0)
        print('board safe')
    else:
        #set only that channel to safe settings
        set_GPIO(cell, 'OFF', EN_CHG_GPIO_LIST)
        set_GPIO(cell, 'OFF', EN_DIS_GPIO_LIST)
        set_GPIO(cell, 'LOW', EN_CUR_GPIO_LIST)

if __name__ == "__main__":
    #run initial setup
    init_gpio()
    safe_board()
    ser.write('SYST:REM\n'.encode())
    ser.write('Syst:beep\n'.encode())
    ser.write('OUTP ON\n'.encode())
    
    #TODO check memory to restart experiment at midpoint
    cycle_count = [0, 20, 20]
    cell_mode = ['CYCLE','IDLE','IDLE'] #three possibilities: CYCLE, TEST, IDLE
    cell_state = ['CHG','REST','REST'] #three possibilities: CHG, DIS, REST
    
    time_init = time.monotonic()
    time_iter = time_init
    time_prev = time_iter
    
    
    #check initial state of batteries
    sensor_data = ping_sensors(cell_state[0])
    log_sensor_data(time_iter, sensor_data)

    
    while True:
        time_iter = time.monotonic()

        # check safe temperatures
        temp_data = sensor_data[0:3]
        for i in range(1):
            if temp_data[i] > TEMP_MAX or temp_data[i] < TEMP_MIN:
                cell_mode[i] = 'IDLE'
                cell_state[i] = 'IDLE'
                safe_board(i)
        
        if time_iter > time_prev + DT_LOG:
            sensor_data = ping_sensors(cell_state[0])
            log_sensor_data(time_iter, sensor_data)
            temp_i = sensor_data[0:3]
            volt_i = sensor_data[4:7]
            curr_i = sensor_data[8:11]
            
            for i in range(2):
                # mode logic
                if cycle_count[i] < 20:
                    cell_mode[i] = 'CYCLE'
                    if cell_state[i] == 'CHG' and volt_i[i] < V_CHG_LIMIT:
                        cell_state[i] = 'CHG'
                    elif cell_state[i] == 'CHG' and volt_i[i] >= V_CHG_LIMIT:
                        cell_state[i] = 'DIS'
                        safe_board(i)
                        print('switch to discharge')
                    elif cell_state[i] == 'DIS' and volt_i[i] > V_DIS_LIMIT:
                        cell_state[i] = 'DIS'
                    elif cell_state[i] == 'DIS' and volt_i[i] <= V_DIS_LIMIT:
                        cell_state[i] = 'CHG'
                        safe_board(i)
                        cycle_count[i] += 1
                        print('switch to charge')
                    else:
                        #print('Idle state')
                        safe_board(i)
                else:
                    cell_mode[i] = 'TEST'
                
                # carry out mode and state we are in
                if cell_mode[i] == 'CYCLE' and cell_state[i]== 'CHG':
                    
                    if temp_i[i] > TEMP_CHG_UPPER:
                        charge_setpoint = CHG_UPPER_SETPT
                    elif temp_i[i] < TEMP_CHG_LOWER:
                        charge_setpoint = CHG_LOWER_SETPT
                    else:
                        charge_setpoint = CHG_LOWER_SETPT + (temp_i[i] - TEMP_CHG_LOWER)*(CHG_UPPER_SETPT - CHG_LOWER_SETPT)/(TEMP_CHG_UPPER - TEMP_CHG_LOWER)
                    
                    if curr_i[i] < charge_setpoint - CHG_SETPT_DELTA:
                        chg_vals[i] += 1
                        set_POT(i, chg_vals[i])
                    elif curr_i[i] > charge_setpoint + CHG_SETPT_DELTA:
                        chg_vals[i] -= 1
                        set_POT(i, chg_vals[i])
                    
                    set_GPIO(i, 'ON', EN_CHG_GPIO_LIST)
                elif cell_mode[i] == 'CYCLE' and cell_state[i] == 'DIS':
                    discharge_setpoint = DIS_SETPT
                    
                    if curr_i[i] < discharge_setpoint - DIS_SETPT_DELTA:
                        dis_vals[i] += 1
                        set_wiper(POT_ADDRS[3], POT_REGS[i], dis_vals[i])
                    elif curr_i[i] > discharge_setpoint + DIS_SETPT_DELTA:
                        dis_vals[i] -= 1
                        set_wiper(POT_ADDRS[3], POT_REGS[i], dis_vals[i])
                    set_GPIO(i, 'ON', EN_DIS_GPIO_LIST)
                    set_GPIO(i, 'HIGH', EN_CUR_GPIO_LIST)
                    print("after seting dis conf")
                    
                elif cell_mode[i] == 'TEST':
                    # do characterization test things
                    #TODO implement characterization test
                    cycle_count[i] = 0
                    cell_mode[i] = 'CYCLE'
                    cell_state[i] = 'CHG'
                    safe_board(i)
                else:
                    #print('Idle state')
                    safe_board(i)
                    # TODO manage exiting IDLE mode
            time_prev = time_iter
            print('heartbeat')
    
        