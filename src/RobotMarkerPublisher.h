#ifndef ROBOT_MARKER_PUBLISHER_H
#define ROBOT_MARKER_PUBLISHER_H

#include <string>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include "AbstractMarkerPublisher.h"

using namespace std;

class RobotMarkerPublisher : public AbstractMarkerPublisher
{
public:
    RobotMarkerPublisher(string topicName, vector<robot_model::JointModelGroup*> groups);

private:
    visualization_msgs::MarkerArray createMarkersForFrame(string frame) override;

    visualization_msgs::Marker createMarker(string frameId);
    visualization_msgs::Marker createMarkerLabel(string frameId, string marker_label);
    visualization_msgs::Marker createMarkerArrow(string frameId);
};

#endif