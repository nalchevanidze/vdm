#ifndef ROBOT_MODEL_TOOLS_H
#define ROBOT_MODEL_TOOLS_H

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>

using namespace std;
using namespace robot_model;
using namespace robot_model_loader;
using namespace robot_state;

/// A tool class containing helpful robot-model-related functions.
class RobotModelTools
{
public:
    /// Returns a vector of all the joint's names contained in the given robot-
    /// model.
    vector<string> getAllJointNames();
};

#endif