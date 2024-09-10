"""
File: LED.py

This module contains a class enabling the 
status or color of an LED peripheral to be changed

Author: Ryan Barry
Date Created: July 16, 2023
"""

from hardware.RoverConstants import OFF, ON


class LED:
    def __init__(self, status=OFF, rgb=False, color=None):
        self.status = status
        self.__rgb = rgb
        if color is not None:
            self.rgb = True
            self.set_color(color)
        self.__switcher = {
            "red": (255, 0, 0),
            "green": (0, 255, 0),
            "blue": (0, 0, 255),
            "white": (255, 255, 255),
            "black": (0, 0, 0),
            "off": (0, 0, 0),
            "yellow": (255, 255, 0),
            "magenta": (255, 0, 255),
            "cyan": (0, 255, 255),
            "orange": (255, 128, 0),
            "pink": (255, 192, 203),
            "purple": (128, 0, 128),
            "lavender": (230, 230, 250),
            "turquoise": (64, 224, 208),
            "gold": (255, 215, 0),
            "silver": (192, 192, 192),
        }

    def on(self):
        self.status = ON

    def off(self):
        self.status = OFF

    def get_color(self, color):
        return self.__switcher[color]

    def set_color(self, color):
        if self.__rgb:
            self.status = color
