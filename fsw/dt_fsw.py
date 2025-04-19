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
import math
import struct
from battery_channel_class import battery_channel
from FSW_PARAMS_class import FSW_PARAMS
from FSW_ADDRS_class import FSW_ADDRS
from packet_handler import (
    handle_request_packet,
    build_combined_packet, buffer_and_log_reading,
)

# Init serial port to communicate with the mock bus
try:
    bus_serial = serial.Serial('/dev/serial0', 115200, timeout=0.1)
    bus_serial.flushInput()
    bus_serial.flushOutput()
    print("[INFO] Bus serial connection established")
except Exception as e:
    print(f"[ERROR] Failed to open serial port for bus: {e}")
    bus_serial = None

# load addresses
ADDRS = FSW_ADDRS()
# load params
PARAMS = FSW_PARAMS()

bus = smbus2.SMBus(1)

file_name = 'ZZ_log_'
trial = 1
while os.path.exists(file_name + str(trial)+ '.csv'):
    trial += 1
file_name = file_name + str(trial) + '.csv'
header = ['Time (s)','Temp0','Temp1','Temp2','Temp_CPU','Volt0','Volt1','Volt2','Volt_CPU','Curr0','Curr1','Curr2','Cyc0','Cyc1','Cyc2','Seq0','Seq1','Seq2',
          'disval0','disval1','disval2','dislowval0','dislowval1','dislowval2','chgval0','chgval1','chgval2','chglowval0','chglowval1','chglowval2',
          'estSOC0','estSOC1','estSOC2','estCAP0','estCAP1','estCAP2','estV0','estV1','estV2']
with open(file_name, 'w', encoding='UTF8', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(header)

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
    GPIO.output(ADDRS.EN_CHG_GPIO_LIST[0], GPIO.HIGH)  #set to LOW, inverted
    GPIO.output(ADDRS.EN_CHG_GPIO_LIST[1], GPIO.HIGH)  #set to LOW, inverted
    GPIO.output(ADDRS.EN_CHG_GPIO_LIST[2], GPIO.HIGH)  #set to LOW, inverted
    GPIO.output(ADDRS.EN_DIS_GPIO_LIST[0], GPIO.LOW)  #set to LOW
    GPIO.output(ADDRS.EN_DIS_GPIO_LIST[1], GPIO.LOW)  #set to LOW
    GPIO.output(ADDRS.EN_DIS_GPIO_LIST[2], GPIO.LOW)  #set to LOW
    GPIO.output(ADDRS.EN_CUR_GPIO_LIST[0], GPIO.LOW)  #set to LOW
    GPIO.output(ADDRS.EN_CUR_GPIO_LIST[1], GPIO.LOW)  #set to LOW
    GPIO.output(ADDRS.EN_CUR_GPIO_LIST[2], GPIO.LOW)  #set to LOW
    GPIO.output(ADDRS.EN_HEATER_GPIO, GPIO.LOW)
    
# def set_GPIO(cell_num, state, GPIO_LIST):
#     # manipulate EN_CHG GPIO pins according to cell num and desired state
#     if state=='ON' or state=='HIGH':
#         GPIO.output(GPIO_LIST[cell_num], GPIO.HIGH) #set HIGH
#     else:
#         GPIO.output(GPIO_LIST[cell_num], GPIO.LOW) #set LOW
            
def set_POT(channel, val):
    # given "10-bit" potentiometer value
    # set each wiper in the potentiometer to the corresponding value
    val = max(0, min(1024, math.floor(val)))
    a = int(val/4)
    b = a + (val%4 > 2)
    c = a + (val%4 > 1)
    d = a + (val%4 > 0)
    set_wiper(ADDRS.POT_ADDRS[channel], ADDRS.POT_REGS[0], a, mode="block")
    set_wiper(ADDRS.POT_ADDRS[channel], ADDRS.POT_REGS[1], b, mode="block")
    set_wiper(ADDRS.POT_ADDRS[channel], ADDRS.POT_REGS[2], c, mode="block")
    set_wiper(ADDRS.POT_ADDRS[channel], ADDRS.POT_REGS[3], d, mode="block")
    
def set_wiper(i2c_address, wiper_register, value, mode="byte"):
    value = math.floor(value)
    if value < 0 or value > 256:
        raise ValueError("Wiper value out of range (0-256)")

    command_byte = (wiper_register << 4) | ((value >> 8) & 0x01)  
    lsb = value & 0xFF  

    try:
        if mode == "byte":
            bus.write_byte_data(i2c_address, wiper_register, lsb)
        elif mode == "block":
            bus.write_i2c_block_data(i2c_address, command_byte, [lsb])
        else:
            raise ValueError("invalid mode")
    except OSError as e:
        print(f"Error setting wiper {wiper_register}: {e}")
        return -1

def read_wiper(i2c_address, wiper_register, mode="byte"):
    try:
        if mode == "byte":
            data = bus.read_byte_data(i2c_address, wiper_register)
            return data
        elif mode == "block":
            command_byte = (wiper_register << 4) | (0x03 << 2)  # read command 11
            data = bus.read_i2c_block_data(i2c_address, command_byte, 2)
            value = ((data[0] & 0x01) << 8) | data[1]  # combine MSB and LSB for 9bit value
            return value
        else:
            raise ValueError("invalid mode")
    except OSError as e:
        print(f"Error reading wiper {wiper_register}: {e}")
        return -1

def read_temperature(channel):
    try:
        # Read two bytes 
        raw_data = bus.read_i2c_block_data(ADDRS.TMP_ADDRS[channel], 0x00, 2)
        
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

def get_CPU_frequency():
    output = subprocess.check_output(['vcgencmd','measure_clock arm'])
    freq = 0  #TODO look at returned statement
    return freq

def read_voltage(channel):
    voltage_raw = bus.read_word_data(ADDRS.INA_ADDRS[channel], 0x02)  #should return 16-bit only positive values
    voltage_raw = ((voltage_raw << 8) & 0xFF00) | (voltage_raw >> 8)
    voltage_v = 1.28 * voltage_raw * 1.25 / 1000  # in Volts
    return voltage_v

def read_current(channel):
    #R_SHUNT_OHMS = 1 #Ohm
    shunt_voltage_raw = bus.read_word_data(ADDRS.INA_ADDRS[channel], 0x01)  #should return 16-bit two's complement
    shunt_voltage_raw = ((shunt_voltage_raw << 8) & 0xFF00) | (shunt_voltage_raw >> 8)
    
    #apply two's complement correction if negative
    # & with 100000000000000 to check if msb is 1 or 0
    if shunt_voltage_raw & (1 << 15):
        shunt_voltage_raw -= (1 << 16)
        
    shunt_voltage_uv = shunt_voltage_raw * 2.5  # in uV
    current_ua = shunt_voltage_uv / PARAMS.R_SHUNT_OHMS[channel]
    current_ma = current_ua / 1000.0
    return current_ma
        
def ping_sensors(ch0, ch1, ch2):
    #read all the sensors
    temp0_c = read_temperature(0)
    temp1_c = read_temperature(1)
    temp2_c = read_temperature(2)
    ch0.temp_c = temp0_c
    ch1.temp_c = temp1_c
    ch2.temp_c = temp2_c
    temp_CPU_c = get_CPU_temperature()
    volt0_v = read_voltage(0)
    volt1_v = read_voltage(1)
    volt2_v = read_voltage(2)
    ch0.volt_v = volt0_v
    ch1.volt_v = volt1_v
    ch2.volt_v = volt2_v
    volt_CPU_v = get_CPU_voltage()
    curr0_ma = read_current(0)
    curr1_ma = read_current(1)
    curr2_ma = read_current(2)
    ch0.curr_ma = curr0_ma
    ch1.curr_ma = curr1_ma
    ch2.curr_ma = curr2_ma
    sensor_data = [temp0_c,temp1_c,temp2_c,temp_CPU_c,volt0_v,volt1_v,volt2_v,volt_CPU_v,curr0_ma,curr1_ma,curr2_ma]
    return sensor_data, ch0, ch1, ch2

def log_sensor_data(time_s, data, ch0, ch1, ch2):
    with open(file_name, 'a', encoding='UTF8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([str(time_s),str(data[0]),str(data[1]),str(data[2]),str(data[3]),str(data[4]),str(data[5]),str(data[6]),str(data[7]),str(data[8]),str(data[9]),str(data[10]),
                         str(ch0.cycle_count),str(ch1.cycle_count),str(ch2.cycle_count),str(ch0.test_sequence),str(ch1.test_sequence),str(ch2.test_sequence),
                         str(ch0.dis_val),str(ch1.dis_val),str(ch2.dis_val),str(ch0.dis_low_val),str(ch1.dis_low_val),str(ch2.dis_low_val),
                         str(ch0.chg_val),str(ch1.chg_val),str(ch2.chg_val),str(ch0.chg_low_val),str(ch1.chg_low_val),str(ch2.chg_low_val),
                         str(ch0.est_soc),str(ch1.est_soc),str(ch2.est_soc),str(ch0.est_capacity_as),str(ch1.est_capacity_as),str(ch2.est_capacity_as),
                         str(ch0.est_volt_v),str(ch1.est_volt_v),str(ch2.est_volt_v)])

def update_actuators(ch):
    #set GPIO first
    if ch.en_dis_state:
        GPIO.output(ADDRS.EN_CHG_GPIO_LIST[ch.channel], GPIO.HIGH) #inverted
        GPIO.output(ADDRS.EN_DIS_GPIO_LIST[ch.channel], GPIO.HIGH)
        if ch.en_cur_state:
            GPIO.output(ADDRS.EN_CUR_GPIO_LIST[ch.channel], GPIO.HIGH) 
        else:
            GPIO.output(ADDRS.EN_CUR_GPIO_LIST[ch.channel], GPIO.LOW) 
    elif ch.en_chg_state:
        GPIO.output(ADDRS.EN_DIS_GPIO_LIST[ch.channel], GPIO.LOW)
        GPIO.output(ADDRS.EN_CUR_GPIO_LIST[ch.channel], GPIO.LOW)
        GPIO.output(ADDRS.EN_CHG_GPIO_LIST[ch.channel], GPIO.LOW) #inverted
    else:
        GPIO.output(ADDRS.EN_DIS_GPIO_LIST[ch.channel], GPIO.LOW)
        GPIO.output(ADDRS.EN_CUR_GPIO_LIST[ch.channel], GPIO.LOW)
        GPIO.output(ADDRS.EN_CHG_GPIO_LIST[ch.channel], GPIO.HIGH) #inverted
    
    #set potentiometers, okay to set pot even if chg/dis circuit isn't active
    if ch.state == 'CHG':
        set_POT(ch.channel, ch.chg_val)                                       # for charging
    if ch.state == 'CHG_LOW':
        set_POT(ch.channel, ch.chg_low_val)
    
    if ch.channel < 0.5:
        if ch.state == 'DIS':
            set_wiper(ADDRS.POT_ADDRS[3], ADDRS.POT_REGS[ch.channel], ch.dis_val, mode="block") # for discharging
        if ch.state == 'DIS_LOW':
            set_wiper(ADDRS.POT_ADDRS[3], ADDRS.POT_REGS[ch.channel], ch.dis_low_val, mode="block")
    else:
        if ch.state == 'DIS':
            set_wiper(ADDRS.POT_ADDRS[3], ADDRS.POT_REGS[ch.channel+1], ch.dis_val, mode="block") # for discharging. Ch1 and Ch2 are on register 2 and 3 
        if ch.state == 'DIS_LOW':
            set_wiper(ADDRS.POT_ADDRS[3], ADDRS.POT_REGS[ch.channel+1], ch.dis_low_val, mode="block") 

def check_for_safety(temp_data_c):
    for i in range(3):
            if temp_data_c[i] > PARAMS.TEMP_MAX_C or temp_data_c[i] < PARAMS.TEMP_MIN_C:
                safe_board(i)

def check_for_data_request():
    return 

def safe_board(cell = -1):
    if cell == -1:
        #set everything to safe settings
        GPIO.output(ADDRS.EN_CHG_GPIO_LIST[0], GPIO.HIGH) #inverted
        GPIO.output(ADDRS.EN_CHG_GPIO_LIST[1], GPIO.HIGH) #inverted
        GPIO.output(ADDRS.EN_CHG_GPIO_LIST[2], GPIO.HIGH) #inverted
        GPIO.output(ADDRS.EN_DIS_GPIO_LIST[0], GPIO.LOW)
        GPIO.output(ADDRS.EN_DIS_GPIO_LIST[1], GPIO.LOW)
        GPIO.output(ADDRS.EN_DIS_GPIO_LIST[2], GPIO.LOW)
        GPIO.output(ADDRS.EN_CUR_GPIO_LIST[0], GPIO.LOW)
        GPIO.output(ADDRS.EN_CUR_GPIO_LIST[1], GPIO.LOW)
        GPIO.output(ADDRS.EN_CUR_GPIO_LIST[2], GPIO.LOW)
        set_wiper(ADDRS.POT_ADDRS[0], ADDRS.POT_REGS[0], 256, mode="block")
        set_wiper(ADDRS.POT_ADDRS[0], ADDRS.POT_REGS[1], 256, mode="block")
        set_wiper(ADDRS.POT_ADDRS[0], ADDRS.POT_REGS[2], 256, mode="block")
        set_wiper(ADDRS.POT_ADDRS[0], ADDRS.POT_REGS[3], 256, mode="block")
        set_wiper(ADDRS.POT_ADDRS[1], ADDRS.POT_REGS[0], 256, mode="block")
        set_wiper(ADDRS.POT_ADDRS[1], ADDRS.POT_REGS[1], 256, mode="block")
        set_wiper(ADDRS.POT_ADDRS[1], ADDRS.POT_REGS[2], 256, mode="block")
        set_wiper(ADDRS.POT_ADDRS[1], ADDRS.POT_REGS[3], 256, mode="block")
        set_wiper(ADDRS.POT_ADDRS[2], ADDRS.POT_REGS[0], 256, mode="block")
        set_wiper(ADDRS.POT_ADDRS[2], ADDRS.POT_REGS[1], 256, mode="block")
        set_wiper(ADDRS.POT_ADDRS[2], ADDRS.POT_REGS[2], 256, mode="block")
        set_wiper(ADDRS.POT_ADDRS[2], ADDRS.POT_REGS[3], 256, mode="block")
        set_wiper(ADDRS.POT_ADDRS[3], ADDRS.POT_REGS[0], 0, mode="block")
        set_wiper(ADDRS.POT_ADDRS[3], ADDRS.POT_REGS[1], 0, mode="block")
        set_wiper(ADDRS.POT_ADDRS[3], ADDRS.POT_REGS[2], 0, mode="block")
        set_wiper(ADDRS.POT_ADDRS[3], ADDRS.POT_REGS[3], 0, mode="block")
        GPIO.output(ADDRS.EN_HEATER_GPIO, GPIO.LOW)
        print('board safe')
    else:
        #set only that channel to safe settings
        GPIO.output(ADDRS.EN_CHG_GPIO_LIST[cell], GPIO.HIGH) #inverted
        GPIO.output(ADDRS.EN_DIS_GPIO_LIST[cell], GPIO.LOW)
        GPIO.output(ADDRS.EN_CUR_GPIO_LIST[cell], GPIO.LOW)

if __name__ == "__main__":
    #run initial setup
    init_GPIO()
    safe_board()
    time.sleep(1)
    
    #TODO check memory to restart experiment at midpoint
    chg_val_prev = -1
    time_init_s = time.monotonic()
    time_iter_s = time_init_s
    time_prev_log_s  = time_iter_s
    time_prev_log2_s = time_iter_s
    time_prev_heat_s = time_iter_s
    time_prev_fast_s = time_iter_s
    time_prev_slow_s = time_iter_s
    time_prev_sensors_s = time_iter_s
    time_prev_check_s = time_iter_s
    
    #create battery channel objects
    ch0 = battery_channel(channel=0,state='CHG',mode='CYCLE',cycle_count=0,volt_v=read_voltage(0),temp_c=read_temperature(0),chg_val=PARAMS.CHG_VAL_INIT,dis_val=PARAMS.DIS_VAL_INIT)
    ch1 = battery_channel(channel=1,state='CHG',mode='CYCLE',cycle_count=0,volt_v=read_voltage(1),temp_c=read_temperature(1),chg_val=PARAMS.CHG_VAL_INIT,dis_val=PARAMS.DIS_VAL_INIT) 
    ch2 = battery_channel(channel=2,state='CHG',mode='CYCLE',cycle_count=0,volt_v=read_voltage(2),temp_c=read_temperature(2),chg_val=PARAMS.CHG_VAL_INIT,dis_val=PARAMS.DIS_VAL_INIT)
    
    # Track last mode switch timestamps per channel
    last_mode_switch = {
        0: time_iter_s,
        1: time_iter_s,
        2: time_iter_s
    }

    # Store current modes for switch detection
    last_mode = {
        0: ch0.mode,
        1: ch1.mode,
        2: ch2.mode
    }

    #check initial state of batteries
    sensor_data, ch0, ch1, ch2 = ping_sensors(ch0, ch1, ch2)
    log_sensor_data(time_iter_s, sensor_data, ch0, ch1, ch2)
    temp_iter_c  = sensor_data[0:3]
    volt_iter_v  = sensor_data[4:7]
    curr_iter_ma = sensor_data[8:11]
    
    try:
        while True:
            time_iter_s = time.monotonic()

            # check safe temperatures, TODO - other checks
            if time_iter_s > time_prev_check_s + PARAMS.DT_CHECK_S:
                check_for_safety(temp_iter_c)
                check_for_data_request()
                time_prev_check_s = time_iter_s

            # Handle incoming data request from bus
            if bus_serial:
                try:
                    waiting_bytes = bus_serial.in_waiting
                
                    if waiting_bytes >= 17:
                        print("[PAYLOAD] Bus request received!")
                        header_and_size = bus_serial.read(12)
                        if header_and_size[:8] != b'\x30\x20\x30\x20\x30\x20\x30\x20':
                            print("[BUS] Invalid packet header received.")
                            continue

                        size = struct.unpack('<I', header_and_size[8:12])[0]
                        rest = bus_serial.read(size)
                        full_packet = header_and_size + rest
                        print("[PAYLOAD] Raw request:", full_packet.hex())
                        response = handle_request_packet(full_packet)
                        if response:
                            bus_serial.write(response)
                            bus_serial.flush()
                            print(f"[BUS] Sent response of length {len(response)} bytes.")
                            print("[BUS DEBUG] Response hex:", response.hex())
                        else:
                            print("[BUS] No response generated.")
                except (OSError, serial.SerialException) as e:
                    print(f"[BUS] Serial error: {e}")

            
            if time_iter_s > time_prev_sensors_s + PARAMS.DT_SENSORS_S:
                sensor_data, ch0, ch1, ch2 = ping_sensors(ch0, ch1, ch2)
                #grab the latest sensor measurements - helpful for some things, debugging
                temp_iter_c  = sensor_data[0:3]
                volt_iter_v  = sensor_data[4:7]
                curr_iter_ma = sensor_data[8:11]
                time_prev_sensors_s = time_iter_s
                
                #check mode and state, update if necessary
                ch0.channel_logic(time_iter_s, PARAMS)
                ch1.channel_logic(time_iter_s, PARAMS)
                ch2.channel_logic(time_iter_s, PARAMS)
                  
                for i, ch in enumerate([ch0, ch1, ch2]):
                    if ch.mode != last_mode[i]:
                        last_mode_switch[i] = time_iter_s
                        last_mode[i] = ch.mode

                time_switches = [
                    int(time_iter_s - last_mode_switch[0]),
                    int(time_iter_s - last_mode_switch[1]),
                    int(time_iter_s - last_mode_switch[2])]
                  
                #overwrite logic for testing  
                  
                #update actuator values for each channel
                ch0.channel_action(PARAMS)
                ch1.channel_action(PARAMS)
                ch2.channel_action(PARAMS)
                
                #overwrite if desired for testing
                # go over range of potentiometer values
                #ch2.chg_val = min(230, max(0, 212+math.floor((time_iter_s - time_init_s)/60)))
                #ch2.state = 'CHG'
                #ch2.en_dis_state = False
                #ch2.en_cur_state = False
                #ch2.en_chg_state = True
                #ch2.update_act = (ch2.chg_val == chg_val_prev)
                #chg_val_prev = ch2.chg_val
                
                #push those actuator values to the board
#                if ch0.update_act:
#                    update_actuators(ch0)
#                if ch1.update_act:
#                    update_actuators(ch1)
#                if ch2.update_act:
#                    update_actuators(ch2)
            
            pulse = ch0.pulse_state or ch1.pulse_state or ch2.pulse_state
            if (time_iter_s > time_prev_log_s + PARAMS.DT_LOG_S) or (pulse and time_iter_s > time_prev_log_s + PARAMS.DT_SENSORS_S):
                #do logging things
                #TODO write to memory to prepare for reset
                log_sensor_data(time_iter_s, sensor_data, ch0, ch1, ch2)
                
                reading_1 = (
                    time_iter_s,
                    ch0.volt_v, ch1.volt_v, ch2.volt_v,
                    ch0.curr_ma, ch1.curr_ma, ch2.curr_ma,
                    ch0.temp_c, ch1.temp_c, ch2.temp_c
                )
                buffer_and_log_reading(1, reading_1)
                
                print('Tempera: %5.2f, %5.2f, %5.2f' % (temp_iter_c[0], temp_iter_c[1], temp_iter_c[2]))
                print('Voltage: %5.2f, %5.2f, %5.2f' % (volt_iter_v[0], volt_iter_v[1], volt_iter_v[2]))
                print('Current: %5.2f, %5.2f, %5.2f' % (curr_iter_ma[0], curr_iter_ma[1], curr_iter_ma[2]))
                print('dis_val: %5.2f, %5.2f, %5.2f' % (ch0.dis_val, ch1.dis_val, ch2.dis_val))
                print('dis_low: %5.2f, %5.2f, %5.2f' % (ch0.dis_low_val, ch1.dis_low_val, ch2.dis_low_val))
                print('chg_val: %5.2f, %5.2f, %5.2f' % (ch0.chg_val, ch1.chg_val, ch2.chg_val))
                print('chg_low: %5.2f, %5.2f, %5.2f' % (ch0.chg_low_val, ch1.chg_low_val, ch2.chg_low_val))
                print('ch_stat:'+ch0.state+','+ch1.state+','+ch2.state)
                print('ch_mode:'+ch0.mode+','+ch1.mode+','+ch2.mode)
                print('Test_sq: %3.1f, %3.1f, %3.1f' % (ch0.test_sequence, ch1.test_sequence, ch2.test_sequence))
                print('SOC_est: %5.2f, %5.2f, %5.2f' % (ch0.est_soc, ch1.est_soc, ch2.est_soc))
                print('CAP est: %5.2f, %5.2f, %5.2f' % (ch0.est_capacity_as, ch1.est_capacity_as, ch2.est_capacity_as))
                time_prev_log_s = time_iter_s
                
            if time_iter_s > time_prev_fast_s + PARAMS.DT_FAST_S:
                #fast loop EKF things, state estimator
                if ch0.mode == 'CYCLE':
                    ch0.state_estimate(volt_iter_v[0], curr_iter_ma[0], time_iter_s)
                if ch1.mode == 'CYCLE':
                    ch1.state_estimate(volt_iter_v[1], curr_iter_ma[1], time_iter_s)
                if ch2.mode == 'CYCLE':
                    ch2.state_estimate(volt_iter_v[2], curr_iter_ma[2], time_iter_s)
                
                reading_3 = (
                    time_iter_s,
                    ch0.est_soc, ch0.est_volt_v, ch0.est_cov_state[0, 0], ch0.est_cov_state[1, 1], ch0.est_capacity_as, ch0.est_cov_param,
                    ch1.est_soc, ch1.est_volt_v, ch1.est_cov_state[0, 0], ch1.est_cov_state[1, 1], ch1.est_capacity_as, ch1.est_cov_param,
                    ch2.est_soc, ch2.est_volt_v, ch2.est_cov_state[0, 0], ch2.est_cov_state[1, 1], ch2.est_capacity_as, ch2.est_cov_param,
                )
                buffer_and_log_reading(3, reading_3)

                #print('heartbeat, fast loop')
                time_prev_fast_s = time_iter_s
                
            if time_iter_s > time_prev_log2_s + PARAMS.DT_LOG2_S:
                reading_2 = (
                    time_iter_s,
                    ch0.cycle_count, ch1.cycle_count, ch2.cycle_count,
                    0,  # replace with actual reset count
                    time_switches[0], time_switches[1], time_switches[2],
                    ch0.test_sequence, ch1.test_sequence, ch2.test_sequence,
                    ch0.state, ch1.state, ch2.state,
                    ch0.mode, ch1.mode, ch2.mode,
                    get_CPU_temperature(),
                    get_CPU_voltage()
                )
                buffer_and_log_reading(2, reading_2)


            if time_iter_s > time_prev_heat_s + PARAMS.DT_HEAT_S:
                #check if heater should be on
                if statistics.median(temp_iter_c) < PARAMS.TEMP_HEATER_ON_C:
                    GPIO.output(ADDRS.EN_HEATER_GPIO, GPIO.HIGH)
                elif statistics.median(temp_iter_c) > PARAMS.TEMP_HEATER_OFF_C:
                    GPIO.output(ADDRS.EN_HEATER_GPIO, GPIO.LOW)
                
                #print('heartbeat, heater')
                time_prev_heat_s = time_iter_s
    
    except (KeyboardInterrupt):
        print('stopping test')
        safe_board()
        
