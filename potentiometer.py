import smbus
import time

# MCP4452 I2C Address
POT1_ADDR = 0x2C  # U2 - GND, GND
POT2_ADDR = 0x2E  # U6 - 3V3, GND
POT3_ADDR = 0x2D  # U3 - GND, 3V3
POT4_ADDR = 0x2F  # U7 - 3V3, 3V3

# MCP4452 Wiper Registers
WIPER_0 = 0x00  
WIPER_1 = 0x01  
WIPER_2 = 0x06  
WIPER_3 = 0x07  

bus = smbus.SMBus(1) #I2C bus 1 on R Pi

def set_wiper(wiper_register, value):
    """ Set MCP4452 wiper position (0-257) """
    if value < 0 or value > 255:
        raise ValueError("Wiper value out of range (0-255)")

    msb = (value >> 8) & 0x01  
    lsb = value & 0xFF         

    bus.write_i2c_block_data(POT1_ADDR, wiper_register, [msb, lsb])
    print(f"Wiper {hex(wiper_register)} set to {value}")

def read_wiper(wiper_register):
    """ Read current MCP4452 wiper position """
    data = bus.read_i2c_block_data(POT1_ADDR, wiper_register, 2)
    value = ((data[0] & 0x01) << 8) | data[1]
    return value

try:
    print("Testing MCP4452 Digital Potentiometer...")

    test_values = [0, 64, 128, 192, 257]  # resistance levels
    

    for value in test_values:
        print(f"\nSetting Wiper 0 to {value}")
        set_wiper(WIPER_0, value)
        time.sleep(1)  

        read_value = read_wiper(WIPER_0)
        print(f"Read Wiper 0 Value: {read_value}")

    print("\nTest completed successfully")

except KeyboardInterrupt:
    print("\nTest interrupted")
finally:
    bus.close()
