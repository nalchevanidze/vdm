#ifndef ROBOT_MARKER_PUBLISHER_H
#define ROBOT_MARKER_PUBLISHER_H


#include <string>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>


using namespace std;


/// This class can be used to generate markers to be displayed in rviz.
class RobotMarkerGenerator 
{
    public:
        /// Constructor
        RobotMarkerGenerator();

        /// Creates a marker array that visualizes the given velocity in form
        /// of an arrow and a text-label.
        ///
        /// The text simply displays the given value.
        /// The arrow-shaped marker visualizes the value and varies in size
        /// depending on the value.
        ///
        /// TODO: Pass in additional prameters for the name and unit.
        visualization_msgs::MarkerArray createVelocityMarkers(
            string frameId, 
            double value
        );

    private:
        /// Generates an id from a given joint-name
        ///
        /// This function works similarly to a hash function in a way that the
        /// same joint-name will always produce the same id.
        ///
        /// This way the visualization marker for a joint can easily be updated
        /// since a new marker array can be created just by calling 
        /// 'createVelocityMarkers()' and publishing the marker-array again.
        int generateIdFromJointName(string jointName);
        
        /// Creates a bare-bones marker attached to the given frame.
        ///
        /// This raw marker can then be further customized.
        visualization_msgs::Marker createMarker(string frameId);

        /// Creates a text-label-marker attached to the given frame, displaying 
        /// the given message.
        visualization_msgs::Marker createMarkerLabel(
            string frameId, 
            string marker_label
        );

        /// Creates an arrow-shaped marker.
        visualization_msgs::Marker createMarkerArrow(string frameId);
};

#endif