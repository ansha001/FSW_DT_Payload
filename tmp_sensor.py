import smbus2
import time

bus = smbus2.SMBus(1)

TMP1 = 0x48  

def read_temperature():
    try:
        # Read two bytes 
        raw_data = bus.read_i2c_block_data(TMP1, 0x00, 2)
        
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

if __name__ == "__main__":
    while True:
        temp = read_temperature()
        if temp is not None:
            print(f"Temperature from TMP112: {temp:.2f}°C")
        time.sleep(5)  
