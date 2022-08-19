#!/home/python_envs/ros-melodic/bin/python3

# State ESTOP or DEFAULT
# Subscribe to bumper to send robot into estop state
# Subscribe to B0 to take robot out of estop state
# Subscribe to /estop/cmd_vel to moderate velocity messages
# Publish to /mobile_base/commands/velocity
# Publish to LED1 to light up led so you know it is in estop state

import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent, ButtonEvent, Led
from enum import Enum

class EStopState(Enum):
    DEFAULT = 0
    ESTOP = 1

class EStopNode():
    def __init__(self) -> None:
        rospy.init_node('estop')
        
        self.vel_sub = rospy.Subscriber('/estop/cmd_vel', Twist, self.velocity_callback)
        self.vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

        self.bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumper_callback)
        self.button_sub = rospy.Subscriber('/mobile_base/events/button', ButtonEvent, self.button_callback)

        self.led_pub = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=10)

        self.state = EStopState.DEFAULT.value

    def velocity_callback(self, twist_msg: Twist)->None:
        # If Robot is not estopped. Forward message to command velocity
        if self.state == EStopState.DEFAULT.value:
            self.vel_pub.publish(twist_msg)
        
        # If Robot is estopped. Do not forward message
        return None
    
    def bumper_callback(self, bumper_msg: BumperEvent)->None:
        # Send the robot into estop state if bumper was hit
        if bumper_msg.state == 1:
            self.state = EStopState.ESTOP.value
        return None

    def button_callback(self, button_msg: ButtonEvent)->None:
        # Take robot out of estop state if button 0 was hit
        if button_msg.button == 0 and button_msg.state == 1:
            self.state = EStopState.DEFAULT.value
        return None
    
    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Update LED
            led_msg = Led()
            if self.state == EStopState.DEFAULT.value:
                led_msg.value = 1
            else:
                twist_msg = Twist()
                self.vel_pub.publish(twist_msg)
                led_msg.value = 3
            self.led_pub.publish(led_msg)
            r.sleep()

if __name__ == '__main__':
    estop = EStopNode()
    estop.run()