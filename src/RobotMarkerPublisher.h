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

    void publishMarker(string frame_id, int id);
};

#endif