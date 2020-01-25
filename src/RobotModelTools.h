#ifndef ROBOT_MODEL_TOOLS_H
#define ROBOT_MODEL_TOOLS_H

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

using namespace std;

/// A tool class containing helpful robot-model-related functions.
class RobotModelTools
{
public:
    /// Returns a vector of all the joint's names contained in the given robot-
    /// model.
    vector<string> getAllJointNames(robot_model::RobotModelPtr model);
};

#endif