#ifndef ROBOT_MARKER_PUBLISHER_H
#define ROBOT_MARKER_PUBLISHER_H

#include <string>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

using namespace std;

class RobotMarkerGenerator 
{
    public:
        RobotMarkerGenerator();

        visualization_msgs::MarkerArray create(string name, string value);

        void reset(); 

    private:
        int idCounter;
        
        visualization_msgs::Marker createMarker(string frameId);
        visualization_msgs::Marker createMarkerLabel(string frameId, string marker_label);
        visualization_msgs::Marker createMarkerArrow(string frameId);
};

#endif