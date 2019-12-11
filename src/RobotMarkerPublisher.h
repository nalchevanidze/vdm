#ifndef ROBOT_MARKER_PUBLISHER_H
#define ROBOT_MARKER_PUBLISHER_H

#include <string>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

using namespace std;

class RobotMarkerPublisher 
{
public:
    RobotMarkerPublisher();
    void startPublishing(vector<robot_model::JointModelGroup*> groups);

private:
    ros::Publisher publisher;
    int idCounter; 

    void publishMarker(string frameId);

    visualization_msgs::Marker createMarker (string frameId);
    visualization_msgs::Marker createLabel(string frameID, string label);
    visualization_msgs::Marker createArrow(string frameID);
};

#endif