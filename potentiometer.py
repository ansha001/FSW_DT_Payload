import smbus2
import time

# MCP4452 I2C Address
POT1_ADDR = 0x2C  # U2 - GND, GND
POT2_ADDR = 0x2E  # U6 - 3V3, GND
POT3_ADDR = 0x2D  # U3 - GND, 3V3
POT4_ADDR = 0x2F  # U7 - 3V3, 3V3

#Decimal addresses
POT1= 44  


# MCP4452 Wiper Registers
WIPER_0 = 0x00  
WIPER_1 = 0x01  
WIPER_2 = 0x06  
WIPER_3 = 0x07  

bus = smbus.SMBus(1) #I2C bus 1 on R Pi

# def setPOS(uint8_t i2cADR, uint8_t ch, uint8_t val):
    

def set_wiper_byte(wiper_register, value):
    if value < 0 or value > 255:
        raise ValueError("Wiper value out of range (0-255)")
    try:
        bus.write_byte_data(POT1_ADDR, wiper_register, value)
        print(f"Wiper {hex(wiper_register)} set to {value}")
    except OSError as e:
        print("Error setting wiper val")
        return -1
    
def set_wiper_block(wiper_register, value):
    if value < 0 or value > 255:
        raise ValueError("out of range (0-255)")
    command_byte = (wiper_register << 4) | ((value >> 8) & 0x01)  # Include D8 bit
 
    lsb = value & 0xFF         

    bus.write_i2c_block_data(POT1_ADDR, command_byte, [lsb])
    print(f"Wiper {hex(wiper_register)} set to {value}")
    
def read_wiper(wiper_register):
        command_byte = (wiper_register << 4) | ((value >> 8) & 0x01)  # Include D8 bit

    try:
        data = bus.read_i2c_block_data(POT1_ADDR, command_byte, 16)
        return value
    except OSError as e:
        print("error reading")
        return -1
try:
    print("Testing MCP4452 Digital Potentiometer...")

    test_values = [0, 64, 128, 192, 256]  
    

    for value in test_values:
        print(f"\nSetting Wiper 0 to {value}")
        set_wiper_block(WIPER_0, value)
#        set_wiper_block(WIPER_1, value)
#        set_wiper_block(WIPER_2, value)
 #       set_wiper_block(WIPER_3, value) 
        time.sleep(1)  

        read_value1 = read_wiper(WIPER_0)
        read_block = bus.read_i2c_block_data(POT1, 0, 16)
  #      read_value2 = read_wiper(WIPER_1)
   #     read_value3 = read_wiper(WIPER_2)
    #    read_value4 = read_wiper(WIPER_3)
        print(f"Read Wiper 0 Value: {read_value1}")
     #   print(f"Read Wiper 1 Value: {read_value2}")
      #  print(f"Read Wiper 2 Value: {read_value3}")
       # print(f"Read Wiper 3 Value: {read_value4}")

    print("\nTest completed successfully")

except KeyboardInterrupt:
    print("\nTest interrupted")
finally:
    bus.close()
