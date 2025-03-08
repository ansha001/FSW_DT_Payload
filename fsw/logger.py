import csv
import time
import os

class DataLogger:
    def __init__(self):
        self.file_name = self.generate_filename()

    def generate_filename(self):
        base_name = "ZZ_log_"
        trial = 1
        while os.path.exists(base_name + str(trial) + ".csv"):
            trial += 1
        return base_name + str(trial) + ".csv"

    def log_data(self, timestamp, data):
        with open(self.file_name, 'a', encoding='UTF8', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([timestamp] + data)
