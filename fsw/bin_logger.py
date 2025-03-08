import struct
import os
import time

# Constants for binary log format
STRUCT_FORMAT = "d 11f"  # d=double (timestamp), 11f = 11 float values (for sensor data)
# 11 : TEMP 012 (3), TEMP CPU, VOLT 012 (3), VOLT CPU, CURR 012 (3)

LOG_MARKER = b"LOG_START\n"  # File marker for easier debugging
LOG_DIR = "sensor_logs" 

class DataLogger:
    def __init__(self):
        os.makedirs(LOG_DIR, exist_ok=True)  # ensure log directory exists
        self.file_name = self._generate_unique_filename()
        self._init_log_file()

    def _generate_unique_filename(self):
        """Generate a unique filename using timestamp"""
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        return os.path.join(LOG_DIR, f"sensor_log_{timestamp}.dat")

    def _init_log_file(self):
        """Create the log file and write an initial marker"""
        try:
            with open(self.file_name, "wb") as f: # wb - fresh write into file
                f.write(LOG_MARKER)
        except Exception as e:
            print(f"Error initializing log file: {e}")

    def log_data(self, timestamp, data):
        """Write timestamped sensor data to the binary log file"""
        if not isinstance(data, (list, tuple)) or len(data) != 11:
            print("Invalid data format! Expected 11 float values.")
            return

        try:
            with open(self.file_name, "ab") as f: # ab - append
                packed_data = struct.pack(STRUCT_FORMAT, timestamp, *data) 
                f.write(packed_data)
        except Exception as e:
            print(f"Error writing to log file: {e}")

    def read_data(self):
        """Read and decode binary log data"""
        try:
            with open(self.file_name, "rb") as f:
                if f.read(len(LOG_MARKER)) != LOG_MARKER:
                    print("Warning: Log file does not contain expected marker!")

                while chunk := f.read(struct.calcsize(STRUCT_FORMAT)):
                    unpacked_data = struct.unpack(STRUCT_FORMAT, chunk)
                    print(f"Time: {unpacked_data[0]:.3f} | Sensors: {unpacked_data[1:]}")
        except FileNotFoundError:
            print("Log file not found")
        except Exception as e:
            print(f"Error reading log file: {e}")
