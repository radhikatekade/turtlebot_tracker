#!/home/python_envs/ros-melodic/bin/python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped


class MainNode():
    def __init__(self):
        rospy.init_node('main')

        # Publishers for aruco marker poses
        self.start_publisher = rospy.Publisher('/aruco_single/pose/start', PoseStamped, queue_size = 10)
        self.tracker_publisher = rospy.Publisher('/aruco_single/pose/tracker', PoseStamped, queue_size=10)
        self.stop_publisher = rospy.Publisher('/aruco_single/pose/stop', PoseStamped, queue_size=10)
         
        # Track which messages we've sent
        # msg0 =  

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            pass

if __name__ == '__main__':
    print("fake_aruco_publisher.py not yet implemented")
    main_node = MainNode()
    main_node.run()

