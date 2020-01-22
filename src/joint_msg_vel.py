#!/usr/bin/env python2

import rospy
import copy
from sensor_msgs.msg import JointState


old_msg = None
rospy.init_node("joint_vel_pub")
publisher = rospy.Publisher("/joint_states", JointState, queue_size=1, tcp_nodelay=True)
def cb(msg):
    global old_msg
    new_msg = copy.copy(msg)
    if old_msg is not None:
        t1 = msg.header.stamp.to_sec()
        t2 = old_msg.header.stamp.to_sec()
        dur = t1- t2
        new_msg.velocity = list()
        for i in range(len(msg.position)):
            new_msg.velocity.append((msg.position[i] - old_msg.position[i]) / dur)
        publisher.publish(new_msg)
    old_msg = msg
            

sub = rospy.Subscriber("/joint_states_wo_vel", JointState, cb)
rospy.spin()