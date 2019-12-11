#include <moveit/robot_model/robot_model.h>

#include "RobotMarkerPublisher.h"


int idCounter = 0;

RobotMarkerPublisher::RobotMarkerPublisher()
{
    ros::NodeHandle node_handle;
    publisher = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 0);
    ros::AsyncSpinner spinner(1);
    spinner.start();
}

void RobotMarkerPublisher::startPublishing(vector<robot_model::JointModelGroup*> groups)
{
    ros::Rate r(100);

    while (ros::ok())
    {
        for (int i = 0; i < groups.size(); i++)
        {
            robot_model::JointModelGroup *currentJointGroup = groups[i];
            vector<string> jointNames = currentJointGroup->getLinkModelNames();
            
            for (int j = 0; j < jointNames.size(); j++) 
            {
                string name = jointNames[j]; 

                // TODO: better id generation
                publishMarker(name);
            }

        }
        r.sleep();
    }
}

// TODO: create dedicated class
visualization_msgs::Marker createMarker(string frameId)
{

    visualization_msgs::Marker marker;

    marker.header.frame_id = frameId;

    marker.header.stamp = ros::Time();
    marker.ns = "stats";

    idCounter++; 
    
    marker.id = idCounter;

    marker.action = visualization_msgs::Marker::ADD;
    // marker.pose.position.x = 1;
    // marker.pose.position.y = 1;
    // marker.pose.position.z = 1;
    // marker.pose.orientation.x = 0.0;
    // marker.pose.orientation.y = 0.0;
    // marker.pose.orientation.z = 0.0;
    // marker.pose.orientation.w = 1.0;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;    

    return marker;
}


visualization_msgs::Marker createMarkerLabel(string frameId, string marker_label)
{
    visualization_msgs::Marker marker;

    marker = createMarker(frameId);
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    marker.text = marker_label;

    marker.scale.x = 0.1;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;
    
    return marker;
}


visualization_msgs::Marker createMarkerArrow(string frameId)
{
    visualization_msgs::Marker marker;

    marker = createMarker(frameId);
    marker.type = visualization_msgs::Marker::ARROW;

    marker.scale.x = 0.1;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;

    return marker;
}



void RobotMarkerPublisher::publishMarker(string frameId)
{
    visualization_msgs::MarkerArray markers;

    visualization_msgs::Marker marker = createMarkerArrow(frameId);  
    visualization_msgs::Marker markerLabel = createMarkerLabel(frameId, "some Jacobian");

    // markers.push_back(marker);
    // markers.push_back(markerLabe);

    publisher.publish(markerLabel);
    publisher.publish(marker);
}  
