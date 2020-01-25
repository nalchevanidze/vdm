#include <ctime>
#include <ratio>
#include <chrono>

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include "RobotModelTools.h"
#include "RobotMarkerGenerator.h"
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

int main(int argc, char** argv) {
    ros::init(argc, argv, "jacobian_calculator");

    ros::NodeHandle n;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);


    auto publisher = n.advertise<visualization_msgs::MarkerArray>("/vdm_markers", 0);
    RobotMarkerGenerator markerGenerator = RobotMarkerGenerator();


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
        int jointIndex = 0;
        markerGenerator.idCounter = 0;
        
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

                
                double velocity = calculateVelocity(currentTimePoint, currentCoordinates, lastTimePoints[jointIndex], lastCoordinates[jointIndex]);
                ROS_ERROR_STREAM("Velocity: " << velocity);


                lastTimePoints[jointIndex] = currentTimePoint;
                lastCoordinates[jointIndex] = currentCoordinates;

                publisher.publish(markerGenerator.createMarkersForFrame(name, to_string(velocity)));

                jointIndex++;
            }
        }


        rate.sleep();
    }
}
