#ifndef VELOCITY_CALCULATOR_H
#define VELOCITY_CALCULATOR_H

#include <string>
#include <ctime>
#include <ratio>
#include <chrono>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

using namespace std;

class VelocityCalculator
{
public:
    double velocityByJointName(string name);

private:
   tf2_ros::Buffer *tfBuffer;
   tf2_ros::TransformListener *tfListener;

    chrono::high_resolution_clock::time_point lastTimePoint;
    vector<double> lastCoordinates;

    double calculateVelocity(
        chrono::high_resolution_clock::time_point t1, 
        vector<double> pos1, 
        chrono::high_resolution_clock::time_point t2, 
        vector<double> pos2
    );
};

#endif