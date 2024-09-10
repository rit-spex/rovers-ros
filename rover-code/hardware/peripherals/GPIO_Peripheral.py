# Import the PYNQ library and the base overlay
"""
File: GPIO_Peripheral.py

Description: This file defines a GPIO_Peripheral class that be used for RPiHat or PMod headers.
The Kria KR260 uses the Petalinux framework for GPIO regulation. PYNQ includes a base 
overlay for the KR260 that handles petalinux natively in Python enabling script-based
GPIO control.

Author: Ryan Barry
Date Created: September 18, 2023
"""
# from pynq.lib import Rpi
from pynq import GPIO

# Load the base overlay
# base = pynq.Overlay("base.bit")

# Create an instance of the RPi interface
# rpi = base.rpi

# Define a class for GPIO pins
class GPIO_Peripheral:
    # Initialize the GPIO object with a pin number and a direction
    def __init__(self, pin, direction):
        self.pin = pin
        self.direction = direction
        self.gpio = GPIO(gpio_index=self.pin, direction=self.direction)
        

    # Write a value (0 or 1) to the GPIO pin
    def write(self, value):
        self.gpio.write(value)

    # Read a value (0 or 1) from the GPIO pin
    def read(self):
        return self.gpio.read()
    
    
    
# # Define a class for GPIO pins
# class GPIO:
#     # Initialize the GPIO object with a pin number and a direction
#     def __init__(self, pin, direction):
#         print("creating gpio object")
#         # Set the pin number
#         self.pin = pin
#         # Set the pin direction ("in" or "out")
#         self.direction = direction
#         # Set the pin mode using the RPi interface

#         Rpi.set_pin(pin, "gpio")
#         print("setting pin")
#         # Set the pin direction using the RPi interface
#         Rpi.set_direction(pin, direction)

#     # Write a value (0 or 1) to the GPIO pin
#     def write(self, value):
#         # Check if the pin direction is "out"
#         if self.direction == "out":
#             # Write the value using the RPi interface
#             Rpi.write(self.pin, value)
#         else:
#             # Raise an exception if the pin direction is not "out"
#             raise Exception(f"Cannot write to GPIO pin {self.pin} with direction {self.direction}")

#     # Read a value (0 or 1) from the GPIO pin
#     def read(self):
#         # Check if the pin direction is "in"
#         if self.direction == "in":
#             # Read the value using the RPi interface
#             value = Rpi.read(self.pin)
#             # Return the value
#             return value
#         else:
#             # Raise an exception if the pin direction is not "in"
#             raise Exception(f"Cannot read from GPIO pin {self.pin} with direction {self.direction}")


# """
# GPIO.py
# Author: Ryan Barry
# Date: September 19, 2023

# Description:
# This script demonstrates GPIO control using the Kria Robotics Stack (KRS) on the Xilinx Kria KR260 robotics kit.

# Instructions:
# 1. Install Kria Robotics Stack on the KR260.
# 2. Connect the appropriate GPIO pin(s) on the KR260.
# 3. Run this script to control the GPIO pin state.

# Note: Make sure to refer to the KRS documentation for proper pin mappings and GPIO functions specific to the KR260 board.

# """

# import krs.gpio as gpio

# class GPIO:
#     def __init__(self, pin_number):
#         self.pin_number = pin_number

#         # Initialize the GPIO pin
#         gpio.init()
#         gpio.set_pin_direction(self.pin_number, gpio.DIRECTION_OUTPUT)

# def set_state(self, state):
#     # Set the state of the GPIO pin (HIGH or LOW)
#     if state == "HIGH":
#         gpio.set_pin_state(self.pin_number, gpio.STATE_HIGH)
#     elif state == "LOW":
#         gpio.set_pin_state(self.pin_number, gpio.STATE_LOW)

# def c# """
# GPIO.py
# Author: Ryan Barry
# Date: September 19, 2023

# Description:
# This script demonstrates GPIO control using the Kria Robotics Stack (KRS) on the Xilinx Kria KR260 robotics kit.

# Instructions:
# 1. Install Kria Robotics Stack on the KR260.
# 2. Connect the appropriate GPIO pin(s) on the KR260.
# 3. Run this script to control the GPIO pin state.

# Note: Make sure to refer to the KRS documentation for proper pin mappings and GPIO functions specific to the KR260 board.

# """

# import krs.gpio as gpio

# class GPIO:
#     def __init__(self, pin_number):
#         self.pin_number = pin_number

#         # Initialize the GPIO pin
#         gpio.init()
#         gpio.set_pin_direction(self.pin_number, gpio.DIRECTION_OUTPUT)

# def set_state(self, state):
#     # Set the state of the GPIO pin (HIGH or LOW)
#     if state == "HIGH":
#         gpio.set_pin_state(self.pin_number, gpio.STATE_HIGH)
#     elif state == "LOW":
#         gpio.set_pin_state(self.pin_number, gpio.STATE_LOW)

# def cleanup(self):
#     # Clean up and release the GPIO pin
#     gpio.cleanup()
# leanup(self):
#     # Clean up and release the GPIO pin
#     gpio.cleanup()
