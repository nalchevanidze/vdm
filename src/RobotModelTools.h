#ifndef ROBOT_MODEL_TOOLS_H
#define ROBOT_MODEL_TOOLS_H

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

using namespace std;

class RobotModelTools
{
public:
    vector<robot_model::JointModelGroup*> 
    *getChainModelGroups(robot_model::RobotModelPtr model);
};

#endif