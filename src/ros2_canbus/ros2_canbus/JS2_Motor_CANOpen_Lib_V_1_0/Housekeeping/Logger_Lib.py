# logger.py

import logging
from datetime import datetime

# Centralized logging configuration
logging.basicConfig(level=logging.INFO, format="%(message)s")

def log(msg: str, type_level_1: str = "", type_level_2: str = "", type_level_3: str = "", type_level_4: str = ""):
    """Log with UTC timestamp at microsecond precision in brackets."""
    now = datetime.now()
    logging.info(f"[{now:%Y-%m-%d %H:%M:%S.%f}] [{type_level_1}] [{type_level_2}] [{type_level_3}] [{type_level_4}] {msg}")
    
"""
[{now:%Y-%m-%d %H:%M:%S.%f}] - UTC timestamp at microsecond precision
msg - The log message
type_level_1 - Type of message in aspect of Node Name
type_level_2 - Type of message in aspect of CANopen Network
type_level_3 - Type of message in aspect of System Architecture
type_level_4 - Type of message  in aspect of Function that called the log



"""


class Logger():
    def __init__(self, Node_Name: str, Node_ID: int):
        self.Node_Name = Node_Name
        self.Node_ID = Node_ID
        self.Node_Name_ID = f"[{Node_ID}:{Node_Name}]"
        self.IF_PRINT = False
        
        self.ID_SET = False
        if self.Node_ID > 0:
            self.ID_SET = True
            self.IF_PRINT and logging.info(f"Logging initiated for [{Node_ID}:{Node_Name}] -> Id is valid")
            
        else:
            self.ID_SET = False
            self.IF_PRINT and logging.info(f"Logging initiated for [{Node_ID}:{Node_Name}] -> Id not Valid yet")



    def print(self, msg: str, type_level_1: str = "",type_level_2: str = "", type_level_3: str = "", type_level_4: str = ""):
        """Log with UTC timestamp at microsecond precision in brackets."""
        now = datetime.now()
        p = f"[{now:%Y-%m-%d %H:%M:%S.%f}] {self.Node_Name_ID} [{type_level_1}] [{type_level_2}] [{type_level_3}] [{type_level_4}] {msg}"
        logging.info(p)
        return p, now