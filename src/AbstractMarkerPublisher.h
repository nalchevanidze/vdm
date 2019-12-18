#ifndef MARKER_PUBLISHER_INTERFACE_H
#define MARKER_PUBLISHER_INTERFACE_H

#include <string>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

using namespace std;

class AbstractMarkerPublisher
{
public:
    AbstractMarkerPublisher(vector<robot_model::JointModelGroup*> groups);
    void startPublishing();

private:
    vector<robot_model::JointModelGroup*> groups;
    ros::Publisher publisher;
    int idCounter; 

    virtual string getPublisherTopicName();
    virtual visualization_msgs::MarkerArray createMarkersForFrame(string frame) = 0;
};

#endif