#!/home/python_envs/ros-melodic/bin/python3

import rospy
from geometry_msgs.msg import Twist

class FakeVelPublisher():
    def __init__(self) -> None:
        rospy.init_node('fakevelpublisher')
        self.vel_pub = rospy.Publisher('/estop/cmd_vel', Twist, queue_size=10)
    
    def publish_velocity(self)->None:
        twist_msg = Twist()
        twist_msg.linear.x = 0.1
        self.vel_pub.publish(twist_msg)

    def run(self)->None:
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Publish velocity
            self.publish_velocity()
            r.sleep()

if __name__ == '__main__':
    fake_vel_publisher = FakeVelPublisher()
    fake_vel_publisher.run()