#!/home/python_envs/ros-melodic/bin/python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from enum import Enum

class UpdateState(Enum):
    DEFAULT = 0
    WAIT = 1
    FOLLOW = 2
    DETECT = 3

StateDict = {
    UpdateState.DEFAULT.value: "DEFAULT",
    UpdateState.WAIT.value: "WAIT",
    UpdateState.FOLLOW.value: "FOLLOW",
    UpdateState.DETECT.value: "DETECT"
}

class MainNode():
    def __init__(self):
        rospy.init_node('main')

        # Subscribers for aruco marker poses
        rospy.Subscriber('/aruco_single/pose/start', PoseStamped, self.start_callback)
        rospy.Subscriber('/aruco_single/pose/tracker', PoseStamped, self.tracker_callback)
        rospy.Subscriber('/aruco_single/pose/stop', PoseStamped, self.stop_callback)

        # Publisher for velocity
        self.vel_pub = rospy.Publisher('/estop/cmd_vel', Twist, queue_size=10)

        # State of the robot
        self.state = UpdateState.DEFAULT.value

        self.START_id = 0
        self.TRACKER_id = 1
        self.STOP_id = 2

        self.START_present = False
        self.TRACKER_present = False
        self.STOP_present = False

        self.old_state = self.state
        print ("Current State: ", StateDict[self.state])
        # If an arcuo tag is not detected within the specified wait time
        # after it was last detected, then the aruco marker is considered
        # no longer present
        self.aruco_wait_time = 0.1 # s
        self.START_time = rospy.get_time() #rospy.Time.now().sec
        self.TRACKER_time = rospy.get_time() #rospy.Time.now().sec
        self.STOP_time = rospy.get_time() #rospy.Time.now().sec
    
    def start_callback(self, pose_msg):
        self.START_present = True
       # _ = pose_msg
        self.START_time = pose_msg.header.stamp
        return None
    
    def tracker_callback(self, pose_msg):
        self.TRACKER_present = True
        self.TRACKER_position = pose_msg.pose.position
        self.TRACKER_orientation = pose_msg.pose.orientation
        self.TRACKER_time = pose_msg.header.stamp

    def stop_callback(self, pose_msg):
        self.STOP_present = True
       # _ = pose_msg
        self.STOP_time = pose_msg.header.stamp
        return None

    def update_state(self):
        # Update self.state of the robot given the current state

        if (self.state != self.old_state):
            print ("Current State: ", StateDict[self.state])
        
        if (self.state == UpdateState.DEFAULT.value):
            if (self.START_present):
                self.state = UpdateState.WAIT.value
        
        elif (self.state == UpdateState.WAIT.value):
            if (self.TRACKER_present):
                self.state = UpdateState.FOLLOW.value
            elif self.STOP_present:
                self.state = UpdateState.DEFAULT.value
        
        elif self.state == UpdateState.FOLLOW.value:
            if (not self.TRACKER_present) and (not self.STOP_present):
                self.state = UpdateState.DETECT.value
            elif self.STOP_present:
                self.state = UpdateState.DEFAULT.value

        elif self.state == UpdateState.DETECT.value:
            if (self.TRACKER_present):
                self.state = UpdateState.FOLLOW.value
            elif self.STOP_present:
                self.state = UpdateState.DEFAULT.value
        else:
            print ("Invalid State Detected!!")
            self.state = UpdateState.DEFAULT.value

        self.old_state = self.state

    def update_control(self):
        # Use self.state and where the TRACKER marker is detected to determine
        # the twist message to send to the robot
        #if (self.state == UpdateState.FOLLOW.value):
            
        pass

    def Tag_Status(self):
        curr_time = rospy.get_time() #rospy.Time.now().sec
        delta_Trackertime = abs(curr_time - self.TRACKER_time)
        delta_Starttime = abs(curr_time - self.START_time)
        delta_Stoptime = abs(curr_time - self.STOP_time)

        if(delta_Trackertime > self.aruco_wait_time):
            self.TRACKER_present = False

        if(delta_Starttime > self.aruco_wait_time):
            self.START_present = False

        if(delta_Stoptime > self.aruco_wait_time):
            self.STOP_present = False


    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.Tag_Status()
            # Update sensor info from camera
            #self.update_sensors()
            # Turn camera info into usable information
            #   which aruco markers were found and where the TRACKER marker
            #   is if it was found
            # Update state of the robot based on latest sensor info
            self.update_state()
            # Use the current state and current camera info to determine control
            self.update_control()

if __name__ == '__main__':
    main_node = MainNode()
    main_node.run()