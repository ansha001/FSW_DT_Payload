import smbus2
import time

POT1_ADDR = 0x2C  
POT2_ADDR = 0x2E 
POT3_ADDR = 0x2D  
POT4_ADDR = 0x2F

#Decimal addresses
POT1= 44  

# MCP4452 Wiper Registers
WIPER_0 = 0x00
WIPER_1 = 0x01
WIPER_2 = 0x06
WIPER_3 = 0x07

bus = smbus2.SMBus(1)  


def set_wiper(i2c_address, wiper_register, value, mode="byte"):
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
#            bus.write_byte(i2c_address, command_byte)
            data = bus.read_i2c_block_data(i2c_address, command_byte, 2)
            value = ((data[0] & 0x01) << 8) | data[1]  # combine MSB and LSB for 9bit value
            return value
        else:
            raise ValueError("invalid mode")
    except OSError as e:
        print(f"Error reading wiper {wiper_register}: {e}")
        return -1


try:
    test_values = [0, 64, 128, 192, 255]

    for value in test_values:
        print(f"\nset wipers to {value}")

        set_wiper(POT1_ADDR, WIPER_0, value, mode="block")
        set_wiper(POT1_ADDR, WIPER_1, value, mode="block")
        set_wiper(POT1_ADDR, WIPER_2, value, mode="block")
        set_wiper(POT1_ADDR, WIPER_3, value, mode="block")

        time.sleep(0.5)  

        read_value1 = read_wiper(POT1_ADDR, WIPER_0, mode="block")
        read_value2 = read_wiper(POT1_ADDR, WIPER_1, mode="block")
        read_value3 = read_wiper(POT1_ADDR, WIPER_2, mode="block")
        read_value4 = read_wiper(POT1_ADDR, WIPER_3, mode="block")

        print(f"Read Wiper 0 : {read_value1}")
        print(f"Read Wiper 1 : {read_value2}")
        print(f"Read Wiper 2 : {read_value3}")
        print(f"Read Wiper 3 : {read_value4}")
        time.sleep(5)

except KeyboardInterrupt:
    print("\nTest interrupted")
finally:
    bus.close()
