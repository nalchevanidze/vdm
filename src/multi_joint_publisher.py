#!/usr/bin/env python

import math
import time
import rospy
from sensor_msgs.msg import JointState

class MultiJointPublisher:

    """
    Publishes valocity values for an arbitrary list of joints specified via
    the 'joints' parameter.
    
    Please note: It is not (yet) possible to specify own values to publish;
    sine-based values between -1 and 1 are periodically published instead.
    """

    def __init__(self, joints):
        self.joints = joints

        rospy.init_node("multi_joint_publisher")
        self.publisher = rospy.Publisher("/joint_states", JointState, queue_size=1)

        self.publish()


    def publish(self):
        msg = JointState()
        msg.name=self.joints

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            now = rospy.Time().now().to_sec()

            sin = math.sin(now)
            cosine = math.cos(now)

            msg.header.stamp=rospy.get_rostime()
            msg.position=map(lambda _ : sin, self.joints)
            msg.velocity=map(lambda _ : -cosine, self.joints)
            msg.effort=map(lambda _ : -sin, self.joints)

            self.publisher.publish(msg)
            rate.sleep()


if __name__ == "__main__":
    MultiJointPublisher(["joint1", "joint2"])