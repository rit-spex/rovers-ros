"""
File: ScienceCamera.py

Description: This file contains attributes for the rover's Science camera.
Any peripheral related to this camera (servos etc) should be instantiated here.

Author: Ryan Barry
Date Created: August 23, 2023
"""
from hardware.RoverPinout import SCIENCE_CAMERA_INDEX
from hardware.sensors.Camera import Camera


class ScienceCamera(Camera):
    def __init__(self, name="science_camera", camera_index=SCIENCE_CAMERA_INDEX):
        super().__init__(name, camera_index)

    def rotate(self):
        pass
