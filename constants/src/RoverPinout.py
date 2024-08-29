# Drive Motor Pins
from RoverConstants import WHEEL_NAMES

WHEEL_PINS = {
    f"{WHEEL_NAMES[0]}_pwm": 1,
    f"{WHEEL_NAMES[1]}_pwm": 2,
    f"{WHEEL_NAMES[2]}_pwm": 3,
    f"{WHEEL_NAMES[3]}_pwm": 4,
    f"{WHEEL_NAMES[4]}_pwm": 5,
    f"{WHEEL_NAMES[5]}_pwm": 6,
}

# Cameras
ARM_CAMERA_INDEX = 0
FRONT_CAMERA_INDEX = 1
SCIENCE_CAMERA_INDEX = 2
