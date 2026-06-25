# logger.py

import time, os
from datetime import datetime

class Common():
    def __init__(self, Node_Name: str, Node_ID: int):
        self.delay_SDO_S = 0.0005
        self.delay_PDO_S = 0.0002
        self.time_period_S = 0.01
        self.time_period_ms = 1000
        pass

    def time_now(self):
        return datetime.time(), datetime.date()
    
    def delay_SDO(self):
        time.sleep(self.delay_SDO_S)

    def delay_PDO(self):
        time.sleep(self.delay_PDO_S)

    def delay(self, delay_time_s: float):
        time.sleep(delay_time_s)

    def file_navigator(self, folder_name: str, file_name: str):
        BASE_DIR = os.path.dirname(os.path.abspath(__file__))
        path = os.path.abspath(os.path.join(BASE_DIR, 
                                                        "..",
                                                        folder_name,
                                                        file_name))
        if not os.path.isfile(path):
            raise FileNotFoundError(f"File not found: {path}")
        return path
