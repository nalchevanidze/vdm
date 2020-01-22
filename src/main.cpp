#include <ctime>
#include <ratio>
#include <chrono>

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include "RobotModelTools.h"
#include "JacobianCalculator.h"
#include "RobotMarkerPublisher.h"
#include "sensor_msgs/JointState.h"


using namespace std;
using namespace robot_model;
using namespace robot_model_loader;
using namespace robot_state;
using namespace ros;


double calculateVelocity(
    chrono::high_resolution_clock::time_point t1, 
    vector<double> pos1, 
    chrono::high_resolution_clock::time_point t2, 
    vector<double> pos2
)
{
    // Hardcoded Euclidian distance between two 3-dimensional vectors
    double distance = sqrt(pow(pos1[0] - pos2[0], 2.0) + pow(pos1[1] - pos2[1], 2.0) + pow(pos1[2] - pos2[2], 2.0));
    chrono::duration<double, std::milli> timeDiff = t1 - t2;

    // v = s / t
    return (distance / (timeDiff.count()));
}


visualization_msgs::Marker createMarker(string frameId, int id)
{

    visualization_msgs::Marker marker;

    marker.header.frame_id = frameId;

    marker.header.stamp = ros::Time();
    marker.ns = "stats";
    
    marker.id = id;

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

visualization_msgs::Marker createMarkerLabel(string frameId, int id, string marker_label)
{
    visualization_msgs::Marker marker;

    marker = createMarker(frameId, id);
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    marker.text = marker_label;

    marker.scale.x = 0.1;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;
    
    return marker;
}

visualization_msgs::Marker createMarkerArrow(string frameId, int id)
{
    visualization_msgs::Marker marker;

    marker = createMarker(frameId, id);
    marker.type = visualization_msgs::Marker::ARROW;

    marker.scale.x = 0.1;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;

    return marker;
}

visualization_msgs::MarkerArray createMarkersForFrame(string frame, int id, string label)
{
    visualization_msgs::MarkerArray markerArray;
    markerArray.markers.push_back(createMarkerLabel(frame, id, label));
    markerArray.markers.push_back(createMarkerArrow(frame, id * 1000));
    return markerArray;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "jacobian_calculator");

    ros::NodeHandle n;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);


    auto publisher = n.advertise<visualization_msgs::MarkerArray>("/vdm_markers", 0);


    RobotModelLoader robotModelLoader("robot_description");
    RobotModelPtr kinematicModel = robotModelLoader.getModel();

    RobotModelTools robotModelTools;

    const vector<JointModelGroup*>& jointModelGroups = kinematicModel->getJointModelGroups();
    vector<JointModelGroup*> groups = robotModelTools.getChainModelGroups(kinematicModel);


    vector<chrono::high_resolution_clock::time_point> lastTimePoints; // chrono::high_resolution_clock::now();
    vector<vector<double>> lastCoordinates; // { 0.0, 0.0, 0.0 };

    for (int i = 0; i < groups.size(); i++)
    {
        robot_model::JointModelGroup *currentJointGroup = groups[i];
        vector<string> jointNames = currentJointGroup->getLinkModelNames();
        
        for (int j = 0; j < jointNames.size(); j++) 
        {
            lastTimePoints.push_back(chrono::high_resolution_clock::now());
            vector<double> dummyCoordninates = { 0.0, 0.0, 0.0 };
            lastCoordinates.push_back(dummyCoordninates);
        }
    }


    ros::Rate rate(10.0);
    while (n.ok())
    {
        int idCounter = 0;
        for (int i = 0; i < groups.size(); i++)
        {
            robot_model::JointModelGroup *currentJointGroup = groups[i];
            vector<string> jointNames = currentJointGroup->getLinkModelNames();
            
            for (int j = 0; j < jointNames.size(); j++) 
            {
                string name = jointNames[j]; 
                ROS_ERROR_STREAM("Name: " << name);

                chrono::high_resolution_clock::time_point currentTimePoint = chrono::high_resolution_clock::now();

                geometry_msgs::TransformStamped transformStamped;
                try
                {
                    transformStamped = tfBuffer.lookupTransform("base_link", name, ros::Time(0));
                }
                catch (tf2::TransformException &ex) 
                {
                    ROS_WARN("%s",ex.what());
                    ros::Duration(1.0).sleep();
                    continue;
                }


                double x = transformStamped.transform.translation.x;
                double y = transformStamped.transform.translation.y;
                double z = transformStamped.transform.translation.z;

                ROS_ERROR_STREAM("Coordinates: " << "x: " << x << ", y: " << y << ", z: " << z);
                vector<double> currentCoordinates = { x, y, z };

                
                double velocity = calculateVelocity(currentTimePoint, currentCoordinates, lastTimePoints[idCounter], lastCoordinates[idCounter]);
                ROS_ERROR_STREAM("Velocity: " << velocity);


                lastTimePoints[idCounter] = currentTimePoint;
                lastCoordinates[idCounter] = currentCoordinates;

                publisher.publish(createMarkersForFrame(name, idCounter, to_string(velocity)));

                idCounter++;
            }
        }


        rate.sleep();
    }
}
