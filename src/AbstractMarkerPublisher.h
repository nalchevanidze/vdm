#ifndef ABSTRACT_MARKER_PUBLISHER_H
#define ABSTRACT_MARKER_PUBLISHER_H

#include <string>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

using namespace std;

class AbstractMarkerPublisher
{
public:
    AbstractMarkerPublisher(string topicName, vector<robot_model::JointModelGroup*> groups);
    void startPublishing();

protected:
    vector<robot_model::JointModelGroup*> groups;
    ros::Publisher publisher;
    int idCounter; 

    virtual visualization_msgs::MarkerArray createMarkersForFrame(string frame) = 0;
};

#endif