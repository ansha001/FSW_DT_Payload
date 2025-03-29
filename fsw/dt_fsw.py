#code for battery cycling
# AZ, Feb 24

import sys
import os
import serial
import time
import csv
import logging
import smbus2
import RPi.GPIO as GPIO
import subprocess
import statistics
from battery_channel_class import battery_channel
from FSW_PARAMS_class import FSW_PARAMS
from FSW_ADDRS_class import FSW_ADDRS

# I2C addresses
ADDRS = FSW_ADDRS(INA_ADDRS = [0x40, 0x41, 0x43], #INA 0,1,2
                  TMP_ADDRS = [0x48, 0x49, 0x4B], #TMP 0,1,2
                  POT_ADDRS = [0x2C, 0x2E, 0x2D, 0x2F], #POT 0,1,2,3
                  POT_REGS  = [0x00, 0x01, 0x06, 0x07],
                  EN_CHG_GPIO_LIST = [23,24,25], #GPIO 23,24,25 is physical pins 16,18,22
                  EN_DIS_GPIO_LIST = [ 5, 6,13], #GPIO  5, 6,13 is physical pins 29,31,33
                  EN_CUR_GPIO_LIST = [12,16,26], #GPIO 12,16,26 is physical pins 32,36,37
                  EN_HEATER_GPIO   = 18)         #GPIO 18 is physical pin 12

bus = smbus2.SMBus(1)

file_name = 'ZZ_log_'
trial = 1
while os.path.exists(file_name + str(trial)+ '.csv'):
    trial += 1
file_name = file_name + str(trial) + '.csv'
header = ['Time (s)','Temp0','Temp1','Temp2','Temp_CPU','Volt0','Volt1','Volt2','Volt_CPU','Curr0','Curr1','Curr2']
with open(file_name, 'w', encoding='UTF8', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(header)
    
# set other parameters TODO read this from a json file or similar
PARAMS = FSW_PARAMS(DT_LOG_S  = 5,              # time step between logs
                    DT_HEAT_S = 60,             # time step between checking heater
                    DT_FAST_S = 1,              # time step between fast loop of EKF
                    DT_SLOW_S = 1000,           # time step between slow loop of EKF
                    CHG_LIMIT_V = 4.0,          # voltage to stop charging
                    DIS_LIMIT_V = 3.2,          # voltage to stop discharging
                    TIME_TO_REST_S = 300,       # time to rest between charge/discharge, seconds
                    TEMP_MAX_C =  65,           # above this, go to safe mode
                    TEMP_MIN_C = -10,           # below this, go to safe mode
                    TEMP_HEATER_ON_C  = 10,     # below this, turn heater on
                    TEMP_HEATER_OFF_C = 25,     # above this, turn heater off
                    TEMP_CHG_MAX_C   = 60,      # above this, stop charging
                    TEMP_CHG_UPPER_C = 15,      # above this, charge setpoint is -40 mA
                    TEMP_CHG_LOWER_C = 5,       # above this, charge setpoint is -10 mA
                    TEMP_CHG_MIN_C   = 0,       # below this, stop charging
                    CHG_UPPER_SETPT_MA = -40,   # nominal setpoint
                    CHG_LOWER_SETPT_MA = -10,   # cold setpoint
                    CHG_MIN_SETPT_MA   = -8,    # minimum setpoint
                    CHG_SETPT_DELTA_MA = 0.5,   # if charging current is outside bound, increment potentiometer
                    DIS_SETPT_MA = 40,          # discharge setpoint
                    DIS_SETPT_DELTA_MA = 0.5,
                    CHG_VAL_INIT = 9,
                    DIS_VAL_INIT = 76)

def init_GPIO():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(ADDRS.EN_CHG_GPIO_LIST[0], GPIO.OUT)
    GPIO.setup(ADDRS.EN_CHG_GPIO_LIST[1], GPIO.OUT)
    GPIO.setup(ADDRS.EN_CHG_GPIO_LIST[2], GPIO.OUT)
    GPIO.setup(ADDRS.EN_DIS_GPIO_LIST[0], GPIO.OUT)
    GPIO.setup(ADDRS.EN_DIS_GPIO_LIST[1], GPIO.OUT)
    GPIO.setup(ADDRS.EN_DIS_GPIO_LIST[2], GPIO.OUT)
    GPIO.setup(ADDRS.EN_CUR_GPIO_LIST[0], GPIO.OUT)
    GPIO.setup(ADDRS.EN_CUR_GPIO_LIST[1], GPIO.OUT)
    GPIO.setup(ADDRS.EN_CUR_GPIO_LIST[2], GPIO.OUT)
    GPIO.setup(ADDRS.EN_HEATER_GPIO, GPIO.OUT)
    GPIO.output(ADDRS.EN_CHG_GPIO_LIST[0], GPIO.LOW)  #set to LOW
    GPIO.output(ADDRS.EN_CHG_GPIO_LIST[1], GPIO.LOW)  #set to LOW
    GPIO.output(ADDRS.EN_CHG_GPIO_LIST[2], GPIO.LOW)  #set to LOW
    GPIO.output(ADDRS.EN_DIS_GPIO_LIST[0], GPIO.LOW)  #set to LOW
    GPIO.output(ADDRS.EN_DIS_GPIO_LIST[1], GPIO.LOW)  #set to LOW
    GPIO.output(ADDRS.EN_DIS_GPIO_LIST[2], GPIO.LOW)  #set to LOW
    GPIO.output(ADDRS.EN_CUR_GPIO_LIST[0], GPIO.LOW)  #set to LOW
    GPIO.output(ADDRS.EN_CUR_GPIO_LIST[1], GPIO.LOW)  #set to LOW
    GPIO.output(ADDRS.EN_CUR_GPIO_LIST[2], GPIO.LOW)  #set to LOW
    GPIO.output(ADDRS.EN_HEATER_GPIO, GPIO.LOW)
    
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
    temperature_c = float(output[5:9])
    #print('CPU Temp is ',temperature,'*C')
    return temperature_c

def get_CPU_voltage():
    output = subprocess.check_output(['vcgencmd','measure_volts'])
    voltage_v = float(output[5:11])
    #print('CPU Voltage is ',voltage,'V')
    return voltage_v

def read_voltage(INA_ADR = 0x40):
    #voltage = 7 #TODO implement voltage read from INA
    voltage_raw = bus.read_word_data(INA_ADR, 0x02)  #should return 16-bit only positive values
    voltage_raw = ((voltage_raw << 8) & 0xFF00) | (voltage_raw >> 8)
    voltage_v = 1.28 * voltage_raw * 1.25 / 1000  # in Volts
    return voltage_v

def read_current(INA_ADR = 0x40):
    #current = 7 #TODO implment current read from INA
    R_SHUNT_OHMS = 1 #Ohm
    shunt_voltage_raw = bus.read_word_data(INA_ADR, 0x01)  #should return 16-bit two's complement
    shunt_voltage_raw = ((shunt_voltage_raw << 8) & 0xFF00) | (shunt_voltage_raw >> 8)
    shunt_voltage_v = shunt_voltage_raw * 2.5  # in V
    current_ma = 1000*shunt_voltage_v / R_SHUNT_OHMS
    return current_ma
        
def ping_sensors(ch0, ch1, ch2):
    #read all the sensors
    temp0_c = read_temperature(TMP_ADDRS[0])
    temp1_c = read_temperature(TMP_ADDRS[1])
    temp2_c = read_temperature(TMP_ADDRS[2])
    ch0.temp_c = temp0_c
    ch1.temp_c = temp1_c
    ch2.temp_c = temp2_c
    temp_CPU_c = get_CPU_temperature()
    volt0_v = read_voltage(INA_ADDRS[0])
    volt1_v = read_voltage(INA_ADDRS[1])
    volt2_v = read_voltage(INA_ADDRS[2])
    ch0.volt_v = volt0_v
    ch1.volt_v = volt1_v
    ch2.volt_v = volt2_v
    volt_CPU_v = get_CPU_voltage()
    curr0_ma = read_current(INA_ADDRS[0])
    curr1_ma = read_current(INA_ADDRS[1])
    curr2_ma = read_current(INA_ADDRS[2])
    ch0.curr_ma = curr0_ma
    ch1.curr_ma = curr1_ma
    ch2.curr_ma = curr2_ma
    sensor_data = [temp0_c,temp1_c,temp2_c,temp_CPU_c,volt0_v,volt1_v,volt2_v,volt_CPU_v,curr0_ma,curr1_ma,curr2_ma]
    return sensor_data, ch0, ch1, ch2

def log_sensor_data(time_s, data):
    with open(file_name, 'a', encoding='UTF8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([str(time_s),str(data[0]),str(data[1]),str(data[2]),str(data[3]),str(data[4]),str(data[5]),str(data[6]),str(data[7]),str(data[8]),str(data[9]),str(data[10])])

def check_for_safety(temp_data_c, PARAMS):
    for i in range(3):
            if temp_data_c[i] > PARAMS.TEMP_MAX_C or temp_data_c[i] < PARAMS.TEMP_MIN_C:
                cell_mode[i] = 'IDLE'
                cell_state[i] = 'IDLE'
                safe_board(i)

def safe_board(cell = -1):
    if cell == -1:
        #set everything to safe settings
        GPIO.output(ADDRS.EN_CHG_GPIO_LIST[0], GPIO.LOW)
        GPIO.output(ADDRS.EN_CHG_GPIO_LIST[1], GPIO.LOW)
        GPIO.output(ADDRS.EN_CHG_GPIO_LIST[2], GPIO.LOW)
        GPIO.output(ADDRS.EN_DIS_GPIO_LIST[0], GPIO.LOW)
        GPIO.output(ADDRS.EN_DIS_GPIO_LIST[1], GPIO.LOW)
        GPIO.output(ADDRS.EN_DIS_GPIO_LIST[2], GPIO.LOW)
        GPIO.output(ADDRS.EN_CUR_GPIO_LIST[0], GPIO.LOW)
        GPIO.output(ADDRS.EN_CUR_GPIO_LIST[1], GPIO.LOW)
        GPIO.output(ADDRS.EN_CUR_GPIO_LIST[2], GPIO.LOW)
        battery_channel.set_wiper(ADDRS.POT_ADDRS[0], ADDRS.POT_REGS[0], 256, mode="block")
        battery_channel.set_wiper(ADDRS.POT_ADDRS[0], ADDRS.POT_REGS[1], 256, mode="block")
        battery_channel.set_wiper(ADDRS.POT_ADDRS[0], ADDRS.POT_REGS[2], 256, mode="block")
        battery_channel.set_wiper(ADDRS.POT_ADDRS[0], ADDRS.POT_REGS[3], 256, mode="block")
        battery_channel.set_wiper(ADDRS.POT_ADDRS[1], ADDRS.POT_REGS[0], 256, mode="block")
        battery_channel.set_wiper(ADDRS.POT_ADDRS[1], ADDRS.POT_REGS[1], 256, mode="block")
        battery_channel.set_wiper(ADDRS.POT_ADDRS[1], ADDRS.POT_REGS[2], 256, mode="block")
        battery_channel.set_wiper(ADDRS.POT_ADDRS[1], ADDRS.POT_REGS[3], 256, mode="block")
        battery_channel.set_wiper(ADDRS.POT_ADDRS[2], ADDRS.POT_REGS[0], 256, mode="block")
        battery_channel.set_wiper(ADDRS.POT_ADDRS[2], ADDRS.POT_REGS[1], 256, mode="block")
        battery_channel.set_wiper(ADDRS.POT_ADDRS[2], ADDRS.POT_REGS[2], 256, mode="block")
        battery_channel.set_wiper(ADDRS.POT_ADDRS[2], ADDRS.POT_REGS[3], 256, mode="block")
        battery_channel.set_wiper(ADDRS.POT_ADDRS[3], ADDRS.POT_REGS[0], 0, mode="block")
        battery_channel.set_wiper(ADDRS.POT_ADDRS[3], ADDRS.POT_REGS[1], 0, mode="block")
        battery_channel.set_wiper(ADDRS.POT_ADDRS[3], ADDRS.POT_REGS[2], 0, mode="block")
        battery_channel.set_wiper(ADDRS.POT_ADDRS[3], ADDRS.POT_REGS[3], 0, mode="block")
        GPIO.output(EN_HEATER_GPIO, GPIO.LOW)
        print('board safe')
    else:
        #set only that channel to safe settings
        GPIO.output(ADDRS.EN_CHG_GPIO_LIST[cell], GPIO.LOW)
        GPIO.output(ADDRS.EN_DIS_GPIO_LIST[cell], GPIO.LOW)
        GPIO.output(ADDRS.EN_CUR_GPIO_LIST[cell], GPIO.LOW)


if __name__ == "__main__":
    #run initial setup
    init_GPIO()
    safe_board()
    
    #TODO check memory to restart experiment at midpoint
    
    time_init_s = time.monotonic()
    time_iter_s = time_init_s
    time_prev_log_s  = time_iter_s
    time_prev_heat_s = time_iter_s
    time_prev_fast_s = time_iter_s
    time_prev_slow_s = time_iter_s
    
    #create battery channel objects
    ch0 = battery_channel(channel=0, state='CHG', mode='CYCLE', cycle_count=0,
                          volt_v=read_voltage(INA_ADDRS[0]), temp_c=read_temperature(TMP_ADDRS[0]),
                          EN_CHG_GPIO=EN_CHG_GPIO_LIST[0],
                          EN_DIS_GPIO=EN_DIS_GPIO_LIST[0], POT_REG=POT_REGS[0])
    ch1 = battery_channel(channel=1, state='CHG', mode='CYCLE', cycle_count=0,
                          volt_v=read_voltage(INA_ADDRS[1]), temp_c=read_temperature(TMP_ADDRS[1]),
                          EN_CHG_GPIO=EN_CHG_GPIO_LIST[1],
                          EN_DIS_GPIO=EN_DIS_GPIO_LIST[1], POT_REG=POT_REGS[1])
    ch2 = battery_channel(channel=2, state='CHG', mode='CYCLE', cycle_count=0,
                          volt_v=read_voltage(INA_ADDRS[2]), temp_c=read_temperature(TMP_ADDRS[2]),
                          EN_CHG_GPIO=EN_CHG_GPIO_LIST[2],
                          EN_DIS_GPIO=EN_DIS_GPIO_LIST[2], POT_REG=POT_REGS[2])
    
    #check initial state of batteries
    sensor_data, ch0, ch1, ch2 = ping_sensors(ch0, ch1, ch2)
    log_sensor_data(time_iter_s, sensor_data)
    
    while True:
        time_iter_s = time.monotonic()

        # check safe temperatures, TODO - other checks
        temp_iter_c = sensor_data[0:3]
        check_for_safety(temp_iter_c, PARAMS)
        
        if time_iter_s > time_prev_log_s + PARAMS.DT_LOG_S:
            #do logging things
            log_sensor_data(time_iter_s, sensor_data)
            print('heartbeat, logging')
            time_prev_log_s = time_iter
            
        if time_iter_s > time_prev_fast_s + PARAMS.DT_FAST_S:
            sensor_data = ping_sensors(ch0, ch1, ch2)
            
            #check mode and state, update if necessary
            ch0.channel_logic(time_iter_s, PARAMS)
            ch1.channel_logic(time_iter_s, PARAMS)
            ch2.channel_logic(time_iter_s, PARAMS)
                
            #update actuator values for each channel
            ch0.channel_action(PARAMS, ADDRS)
            ch1.channel_action(PARAMS, ADDRS)
            ch2.channel_action(PARAMS, ADDRS)
            
            print('heartbeat, fast loop')
            time_prev_fast_s = time_iter_s
            
            
        if time_iter_s > time_prev_slow_s + PARAMS.DT_SLOW_S:
            #do slow loop EKF things
            print('heartbeat, slow EKF')
            time_prev_slow_s = time_iter_s

        if time_iter_s > time_prev_heat_s + PARAMS.DT_HEAT_S:
            #check if heater should be on
            if statistics.median(temp_iter_c) < PARAMS.TEMP_HEATER_ON_C:
                GPIO.output(EN_HEATER_GPIO, GPIO.HIGH)
            elif statistics.median(temp_iter_c) > PARAMS.TEMP_HEATER_OFF_C:
                GPIO.output(EN_HEATER_GPIO, GPIO.LOW)
            
            print('heartbeat, heater')
            time_prev_heat_s = time_iter_s