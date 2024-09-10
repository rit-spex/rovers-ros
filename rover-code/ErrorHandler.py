"""
File: ErrorHandler.py

Description:The ErrorHandler class provides functionality for logging and handling errors. 
It maintains a dictionary of predefined error messages mapped to error codes. When an error 
occurs, the appropriate error message is logged based on the error code. The class can enable 
logging to write errors to log files, or simply print the errors to the console. 

Author: Ryan Barry
Date Created: August 16, 2023
"""
import datetime
import logging
import os

from hardware.RoverConstants import *

current_file_path = os.path.abspath(__file__)
current_folder_path = os.path.dirname(current_file_path)


class Logger:
    def __init__(self, log_folder):
        self.log_folder = log_folder
        self.logger = None

    def configure_logger(self):
        # Create the log folder if it doesn't exist
        os.makedirs(self.log_folder, exist_ok=True)

        # Get the current date and time
        current_date_time = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

        # Create the log file path using the formatted date and time
        log_file_path = os.path.join(self.log_folder, f"error_log_{current_date_time}.log")

        # Configure logging settings
        self.logger = logging.getLogger()
        self.logger.setLevel(logging.DEBUG)

        # Create a file handler and set the log file path
        file_handler = logging.FileHandler(log_file_path)
        file_handler.setLevel(logging.DEBUG)

        # Create a log format
        log_format = logging.Formatter("%(asctime)s - %(levelname)s - %(message)s")
        file_handler.setFormatter(log_format)

        # Add the file handler to the logger
        self.logger.addHandler(file_handler)

        # Log a message
        self.logger.info("Begin Run Log.")


class ErrorHandler:
    def __init__(self, logging_enabled=False):
        self.error_messages = {MISSION_FAILURE: "CRITICAL ERROR: MISSION FAILURE"}
        self._error = None
        self.logging_enabled = logging_enabled

        if self.logging_enabled:
            log_folder = os.path.join(current_folder_path, "logs")
            self.logger = Logger(log_folder)
            self.logger.configure_logger()

    def error(self, new_error):
        self._error = new_error

    def log_error(self, new_error):
        self.error(new_error)
        if self._error in self.error_messages.keys():
            error_message = self.error_messages[self._error]
        else:
            error_message = f"{new_error}"
        if self.logging_enabled:
            self.logger.logger.error(error_message)
        else:
            date_time = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            print(f"{date_time} - Error - {error_message}")


# if __name__ == "__main__":
#     error_handler = ErrorHandler(logging_enabled=False)
#     error_handler.log_error(MISSION_FAILURE)
