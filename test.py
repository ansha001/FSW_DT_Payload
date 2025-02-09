import sys
import unittest
from unittest.mock import patch

# ================
#  MOCK CLASSES
# ================
class SMBusMock:
    """Mock I2C Bus for Windows testing."""
    def write_byte_data(self, addr, reg, value):
        print(f"Mock I2C Write - Addr: {hex(addr)}, Reg: {hex(reg)}, Value: {value}")

    def read_word_data(self, addr, reg):
        print(f"Mock I2C Read - Addr: {hex(addr)}, Reg: {hex(reg)}")
        return 500  # mock sensor value

class GPIOMock:
    """Mock GPIO for Windows testing."""
    BCM = "BCM"
    OUT = "OUT"
    LOW = "LOW"
    HIGH = "HIGH"

    def setmode(self, mode):
        print("Mock GPIO setmode:", mode)

    def setup(self, pin, mode, initial=None):
        print(f"Mock GPIO setup - Pin: {pin}, Mode: {mode}, Initial: {initial}")

    def output(self, pin, value):
        print(f"Mock GPIO output - Pin: {pin}, Value: {value}")

    def cleanup(self):
        print("Mock GPIO cleanup")

# ===================
#  UNIT TEST CLASS
# ===================
class TestBatteryManager(unittest.TestCase):

    @patch("battery_manager.I2C_BUS", new_callable=lambda: SMBusMock())  # Mock I2C globally
    @patch("battery_manager.GPIO", new_callable=lambda: GPIOMock())  # Mock GPIO globally
    def test_battery_cycle_manager(self, mock_gpio, mock_i2c):
        """Test battery cycle manager with mocked I2C and GPIO."""
        from battery_manager import battery_cycle_manager

        print("\nRunning battery cycle manager test...\n")

        try:
            battery_cycle_manager()  
        except KeyboardInterrupt:
            print("Test stopped manually.")

if __name__ == "__main__":
    unittest.main()