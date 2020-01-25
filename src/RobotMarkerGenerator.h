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
        /// TODO: Convert 'value' to double and pass in additional prameters
        /// for the name and unit.
        visualization_msgs::MarkerArray createVelocityMarkers(string frameId, double value);

        /// Resets the internal id-counter.
        ///
        /// Internally each marker's id is created by using a counter that is
        /// incremented each time 'create()' gets called. This way each marker
        /// is assigned a unique id.
        ///
        /// This function resets that very counter which can be used to
        /// intentionally re-assign ids.
        /// This can be useful in case a bunch of markers is should be
        /// updated periodically from within a for-loop instead of publishing a
        /// new marker every time.
        void reset(); 

    private:
        /// The counter used to determine a marker's id.
        int idCounter;
        
        /// Creates a bare-bones marker attached to the given frame.
        ///
        /// This raw marker can then be further customized.
        visualization_msgs::Marker createMarker(string frameId);

        /// Creates a text-label-marker attached to the given frame, displaying the given
        /// message.
        visualization_msgs::Marker createMarkerLabel(string frameId, string marker_label);

        /// Creates an arrow-shaped marker.
        visualization_msgs::Marker createMarkerArrow(string frameId);
};

#endif