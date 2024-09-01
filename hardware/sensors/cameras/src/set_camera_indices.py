#!/usr/bin/env python3
import rospy
import rospack
import sys

package_path = rospack.get_path('constants')
sys.path.append(package_path + '/src')

from RoverPinout import ARM_CAMERA_INDEX, FRONT_CAMERA_INDEX, SCIENCE_CAMERA_INDEX

def set_camera_indices():
    rospy.set_param('arm_camera_index', ARM_CAMERA_INDEX)
    rospy.set_param('front_camera_index', FRONT_CAMERA_INDEX)
    rospy.set_param('science_camera_index', SCIENCE_CAMERA_INDEX)

if __name__ == "__main__":
    rospy.init_node('set_camera_indices_node', anonymous=True)
    set_camera_indices()
