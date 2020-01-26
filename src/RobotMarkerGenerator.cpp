#include <string>
#include <unordered_map>

#include <moveit/robot_model/robot_model.h>

#include "RobotMarkerGenerator.h"


RobotMarkerGenerator::RobotMarkerGenerator()
{}


visualization_msgs::MarkerArray RobotMarkerGenerator::createVelocityMarkers(
    string frameId, 
    string label,
    double velocity,
    string unit
)
{
    visualization_msgs::MarkerArray markerArray;

    const string formattedVelocity = 
        label + ": " + to_string(velocity) + " " + unit;

    markerArray.markers.push_back(createMarkerLabel(frameId, formattedVelocity));
    markerArray.markers.push_back(createMarkerArrow(frameId, velocity));

    return markerArray;
}


int RobotMarkerGenerator::generateIdFromJointName(string jointName)
{
    hash<string> hasher;
    size_t hash = hasher(jointName);
    int id = static_cast<int>(hash);
    return id;
}


visualization_msgs::Marker RobotMarkerGenerator::createMarker(string frameId)
{
    visualization_msgs::Marker marker;

    marker.id = generateIdFromJointName(frameId);
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

visualization_msgs::Marker RobotMarkerGenerator::createMarkerLabel(
    string frameId, 
    string marker_label
)
{
    visualization_msgs::Marker marker;

    marker = createMarker(frameId);
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    marker.id = marker.id + 1000;

    marker.text = marker_label;

    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;

    marker.scale.x = 0.1;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;
    
    return marker;
}

visualization_msgs::Marker RobotMarkerGenerator::createMarkerArrow(
    string frameId, 
    double velocity
)
{
    visualization_msgs::Marker marker;

    marker = createMarker(frameId);
    marker.type = visualization_msgs::Marker::ARROW;

    marker.id = marker.id + 2000;

    marker.scale.x = velocity * 0.3;
    marker.scale.y = velocity * 0.3;
    marker.scale.z = velocity * 0.1;

    return marker;
}