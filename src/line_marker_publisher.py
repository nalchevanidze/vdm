#!/usr/bin/python

import rospy
from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray, Marker

_MARKER_SCALE_FACTOR = 0.5

class LineMarkerPublisher:

    def __init__(self):
        rospy.init_node("line_marker_publisher")

        self.ma_publisher = rospy.Publisher("/dynamics_markers", MarkerArray, queue_size=1)
        rospy.Subscriber("/joint_states", JointState, self.js_callback)

        rospy.spin()


    def js_callback(self, msg):
        marker_array = MarkerArray()
        
        marker = Marker()
        marker.header.stamp = msg.header.stamp
        marker.header.frame_id = "link1"
        marker.ns = "joint1-markers"
        marker.id = 1000
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = _MARKER_SCALE_FACTOR * msg.velocity[0]
        marker.scale.y = _MARKER_SCALE_FACTOR * 0.2
        marker.scale.z = _MARKER_SCALE_FACTOR * 0.2
        marker.pose.position.y = 2.2
        marker.color.a = 1
        marker.color.g = 1
        
        marker_array.markers.append(marker)
        self.ma_publisher.publish(marker_array)


if __name__ == "__main__":
    LineMarkerPublisher()