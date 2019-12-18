#ifndef JACOBIAN_CALCULATOR_H
#define JACOBIAN_CALCULATOR_H

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

using namespace std;

class JacobianCalculator
{
public:
    string jacobianValueOf(robot_model::JointModelGroup *group, string endpoint);

private:
    Eigen::MatrixXd calculateJacobian(robot_model::JointModelGroup *group, string endpoint);

};

#endif