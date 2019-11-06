#!/usr/bin/python

import rospy
from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray, Marker

_MARKER_SCALE_FACTOR = 0.5

class LineMarkerPublisher:

    """
    Publishes a MarkerArray visualizing the velocity of the moving part of the 
    line.
    """

    def __init__(self):
        rospy.init_node("line_marker_publisher")

        # Creates a Publisher that boradcasts the MarkerArray to a custom
        # topic called '/dynamics_markers'.
        # rviz needs to be set up as a subscriber manually.
        self.ma_publisher = rospy.Publisher("/dynamics_markers", MarkerArray, queue_size=1)
        
        # Subscribe to the '/joint_states' topic and register 'js_callback' as
        # a callback.
        # 'js_callback' gets the position and velocity from each message and
        # publishes a marker visualizing the obtained values. 
        rospy.Subscriber("/joint_states", JointState, self.js_callback)

        # Keep this Publisher alive.
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
        # Set scale
        marker.scale.x = _MARKER_SCALE_FACTOR * msg.velocity[0]
        marker.scale.y = _MARKER_SCALE_FACTOR * 0.2
        marker.scale.z = _MARKER_SCALE_FACTOR * 0.2
        # Position relative to the link
        marker.pose.position.y = 2.2
        # Set color and opacity
        marker.color.g = 1
        marker.color.a = 1
        
        # Publish the MarkerArray
        marker_array.markers.append(marker)
        self.ma_publisher.publish(marker_array)


if __name__ == "__main__":
    LineMarkerPublisher()