#!/usr/bin/env python3
"""
File: SciencePlate.py

Description:

Author: Ryan Barry
Date Created:
"""

from hardware.subsystems.science_plate.Backpack import Backpack
from hardware.subsystems.science_plate.DNADetector import DNADetector
from hardware.subsystems.science_plate.HeatBlock import HeatBlock
from hardware.subsystems.science_plate.LipidDetector import LipidDetector
from hardware.sensors.Fluorometer import Fluorometer
from hardware.sensors.Humidity import Humidity
from hardware.subsystems.cameras.ScienceCamera import ScienceCamera


class SciencePlate:
    def __init__(self):
        print("science plate")
        self.camera = ScienceCamera()
        print("science camera")
        self.fluorometer = Fluorometer()
        print("fluromoeter")
        self.humidity_sensor = Humidity()
        print("humidititty")
        self.heat_block = HeatBlock()
        print("heat block")
        self.backpack = Backpack()
        print("backpack")
        self.lipid_detector = LipidDetector()
        print("lipid detector")
        self.dna_detector = DNADetector()
        print("dna detector")
