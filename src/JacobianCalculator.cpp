#include "JacobianCalculator.h"

#include <string.h>

Eigen::MatrixXd JacobianCalculator::calculateJacobian(robot_model::JointModelGroup *group, string endpoint)
{
    // string endpoint = group->getLinkModelNames().back();
    string groupName = group->getName();

    moveit::planning_interface::MoveGroupInterface current_move_group(groupName);
    robot_model::RobotStatePtr kinematic_state = current_move_group.getCurrentState();
    
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    Eigen::MatrixXd jacobian;
    
    kinematic_state->getJacobian(group, kinematic_state->getLinkModel(endpoint), reference_point_position, jacobian);
    

    // ROS_INFO_STREAM("Calculated Jacobian for JointModelGroup '" << groupName << "':\n");
    // ROS_INFO_STREAM(jacobian << "\n");

    return jacobian;
}


string JacobianCalculator::jacobianValueOf (robot_model::JointModelGroup *group, string endpoint)
{
    stringstream stStream;
    stStream << calculateJacobian(group, endpoint);
    return stStream.str();
}
