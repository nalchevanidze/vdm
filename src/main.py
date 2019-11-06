#!/usr/bin/python

import math
import time
import rospy
from sensor_msgs.msg import JointState

class LineJointStatePublisher:

    """
    Custom joint-state-publisher controlling the movement of our line-model.
    """

    def __init__(self):
        rospy.init_node("line_joint_state_publisher")
        self.js_pub = rospy.Publisher("/joint_states", JointState, queue_size=1)
        
        msg = JointState()
        msg.name=["joint1"]

        while not rospy.is_shutdown():
            sin = 2 * math.sin(time.clock())
            msg.header.stamp=rospy.get_rostime()
            msg.position=[sin]
            self.js_pub.publish(msg)


if __name__ == "__main__":
    LineJointStatePublisher()