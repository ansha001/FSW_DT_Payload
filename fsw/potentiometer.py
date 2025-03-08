from config import bus, POT_ADDRS, POT_REGS

class PotentiometerControl:
    def __init__(self):
        pass

    def set_wiper(self, i2c_address, wiper_register, value):
        print(f"Setting wiper value in wiper {wiper_register} to {value}")
        if not (0 <= value <= 256):
            raise ValueError("Wiper value out of range (0-256)")
        
        command_byte = (wiper_register << 4) | ((value >> 8) & 0x01)
        lsb = value & 0xFF

        try:
            bus.write_i2c_block_data(i2c_address, command_byte, [lsb])
        except OSError as e:
            print(f"Error setting wiper {wiper_register} on {i2c_address}: {e}")

    def set_pot(self, cell_num, val):
        val = max(0, min(1024, val))
        A, B, C, D = [int(val / 4) + (val % 4 > i) for i in range(4)]

        for i, register in enumerate(POT_REGS):
            self.set_wiper(POT_ADDRS[cell_num], register, [A, B, C, D][i])

    def read_wiper(i2c_address, wiper_register):
        try:
            command_byte = (wiper_register << 4) | (0x03 << 2) # command for read is 11(binary) or 0x03(hex)
            data = bus.read_i2c_block_data(i2c_address, command_byte, 2)
            value = ((data[0] & 0x01) << 8) | data[1]  # combine MSB and LSB for 9 bit value
            return value
        except OSError as e:
            print(f"Error reading wiper {wiper_register}: {e}")