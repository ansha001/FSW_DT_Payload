import csv
import time
import serial
import os
import struct

#set test parameters
file_name = 'PS_log_'
trial = 1
while os.path.exists(file_name + str(trial)+ '.csv'):
    trial += 1
file_name = file_name + str(trial) + '.csv'
header = ['Time (s)','Voltage (V)','Current (A)']
with open(file_name, 'w', encoding='UTF8', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(header)

time_final = 10 # length of test, seconds
freq = 1        # desired logging rate, Hz
dt = 1/freq     # desired step size, seconds
    
V_limit = 3.141 # voltage limit, Volts
I_limit = 0.010 # current limit, Amps
V_prev  = -1.00 
I_prev  = -1.00
dV_min  = 0.001 # minimum voltage change to signal change
dI_min  = 0.001 # minimum current change to signal change
    
t_list = []
V_list = []
I_list = []

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

power_supply_inactive = True

if (__name__ == "__main__"):
    #check device connection
    ser.open()
    ser.write('*IDN?\n'.encode())
    ack = ser.readline().decode()
    print(ack)
    ser.write('SYST:REM\n'.encode())
    ser.write('Syst:beep\n'.encode())
    #ser.close()
    
    time_init = time.monotonic()
    time_i 	  = time_init
    time_prev = time_i
    while time_i < time_init + time_final:
        time_i = time.monotonic()
        if time_i > time_prev + dt:
            time_prev = time_i
            # determine voltage and current settings
            ## ADJUST HERE FOR DIFFERENT PROFILES
            ## CONSTANT
            V_limit = V_limit
            I_limit = I_limit  # limits are set above
            ## STEP
#             if time_i < 10:
#                 V_limit = 5.00
#                 I_limit = 0.00
#             else:
#                 V_limit = 5.00  #ADJUST AS NECESSARY
#                 I_limit = 0.50
             
            #limit checking
            if V_limit < 0:
                print('Error, voltage negative')
                V_limit = 0
            elif V_limit > 30:
                print('Error, voltage too high')
                V_limit = 30
            if I_limit < 0:
                print('Error, current negative')
                I_limit = 0
            elif I_limit > 3.0:
                print('Error, current too high')
                I_limit = 3.0
                
            #ser.open() 
            if power_supply_inactive:
                #print('Chan:outp 1'.encode('utf-8'))
                #ser.write('Chan:outp 1'.encode('utf-8'))
                #ser.write('SOUR:CHAN:OUTP:1\n'.encode())
                ser.write('OUTP ON\n'.encode())
                power_supply_inactive = False
                print('power supply on')
            
            #set voltage and current
            if abs(V_limit - V_prev) > dV_min or abs(I_limit - I_prev) > dI_min:
                V_prev = V_limit
                I_prev = I_limit
                V_str = '{0:.2f}'.format(V_limit)
                I_str = '{0:.2f}'.format(I_limit)
                print('SOUR:APP ch1,'+V_str+','+I_str+'\n')
                #ser.write(('SOUR:APP ch1,'+V_str+','+I_str+'\n').encode())
                V_str = 'SOUR:APP:VOLT 3.14,0.00,0.00\n'
                ser.write(V_str.encode())
                ack = ser.readline().decode()
                print(ack)
            
            #log voltage and current
            #print('MEAS:VOLT?')
            ser.write('MEAS:VOLT?\n'.encode())
            V_meas = ser.readline().decode()
            #print(V_meas)
            ser.write('MEAS:CURR?\n'.encode())
            #I_meas = struct.unpack('>f', ser.read(50))
            I_meas = ser.readline().decode()
            print(time_i - time_init, V_meas[0:6], I_meas[0:9])
            #ser.close()
            
            t_list.append(time_i - time_init)
            V_list.append(V_meas[0:6])
            I_list.append(I_meas[0:9])
        #out of if
    #out of while
    
    if not power_supply_inactive:
        #ser.open
        ser.write('Syst:beep\n'.encode())
        ser.write('OUTP OFF\n'.encode())
        ser.close
        power_supply_inactive = True
        print('Power supply off')
        
    
    with open(file_name, 'a', encoding='UTF8', newline='') as f:
        writer = csv.writer(f)
        for i in range(len(t_list)):
            writer.writerow([str(t_list[i]),V_list[i],I_list[i]])