#include <moveit/robot_model/robot_model.h>

#include "RobotMarkerPublisher.h"


RobotMarkerPublisher::RobotMarkerPublisher()
{}

int idCounter = 0;

visualization_msgs::MarkerArray RobotMarkerPublisher::createMarkersForFrame(string frame, string label)
{
    visualization_msgs::MarkerArray markerArray;
    markerArray.markers.push_back(createMarkerLabel(frame, label));
    markerArray.markers.push_back(createMarkerArrow(frame));
    return markerArray;
}

visualization_msgs::Marker RobotMarkerPublisher::createMarker(string frameId)
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
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;    

    return marker;
}

visualization_msgs::Marker RobotMarkerPublisher::createMarkerLabel(string frameId, string marker_label)
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

visualization_msgs::Marker RobotMarkerPublisher::createMarkerArrow(string frameId)
{
    visualization_msgs::Marker marker;

    marker = createMarker(frameId);
    marker.type = visualization_msgs::Marker::ARROW;

    marker.scale.x = 0.1;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;

    return marker;
}