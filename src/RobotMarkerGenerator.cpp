#include <string>

#include <moveit/robot_model/robot_model.h>

#include "RobotMarkerGenerator.h"


RobotMarkerGenerator::RobotMarkerGenerator()
{}

int idCounter = 0;

visualization_msgs::MarkerArray RobotMarkerGenerator::createVelocityMarkers(string frameId, double value)
{
    visualization_msgs::MarkerArray markerArray;

    const string formattedVelocity = to_string(value) + " m/s";
    markerArray.markers.push_back(createMarkerLabel(frameId, formattedVelocity));
    markerArray.markers.push_back(createMarkerArrow(frameId));
    return markerArray;
}

void RobotMarkerGenerator::reset()
{
    idCounter = 0;
}

visualization_msgs::Marker RobotMarkerGenerator::createMarker(string frameId)
{
    idCounter++; 

    visualization_msgs::Marker marker;

    marker.id = idCounter;
    marker.header.frame_id = frameId;
    marker.header.stamp = ros::Time();
    marker.ns = "stats";

    marker.action = visualization_msgs::Marker::ADD;
    // marker.pose.position.x = 1;
    // marker.pose.position.y = 1;
    // marker.pose.position.z = 1;

    // TODO: set orientation by velocity direction
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

visualization_msgs::Marker RobotMarkerGenerator::createMarkerLabel(string frameId, string marker_label)
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

visualization_msgs::Marker RobotMarkerGenerator::createMarkerArrow(string frameId)
{
    visualization_msgs::Marker marker;

    marker = createMarker(frameId);
    marker.type = visualization_msgs::Marker::ARROW;

    marker.scale.x = 0.1;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;

    return marker;
}