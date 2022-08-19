#!/home/python_envs/ros-melodic/bin/python3

import cv2
import rospy

class CameraNode():
    def __init__(self) -> None:
        rospy.init_node('camera')

        self.camera_pub = rospy.Publisher('')