#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

using namespace std;
using namespace robot_model;
using namespace robot_model_loader;
using namespace robot_state;


int main(int argc, char** argv) {
    ros::init(argc, argv, "jacobian_calculator");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    RobotModelLoader robotModelLoader("robot_description");

    RobotModelPtr kinematicModel = robotModelLoader.getModel();


    // model groups
    const vector<JointModelGroup*>& jointModelGroups = kinematicModel->getJointModelGroups();
    
    // filters out only chained gruops
    vector<JointModelGroup*> chainedModelGroups;
    for(int i = 0; i < jointModelGroups.size(); i ++ )
    {
        JointModelGroup *current = jointModelGroups[i];

        if(current->isChain())
        {
            chainedModelGroups.push_back(current);
            ROS_INFO_STREAM(current->getName());
        }
    }

    // filters out only chained gruops
    for(int i = 0; i < chainedModelGroups.size(); i ++ )
    {
        JointModelGroup *currentJointGroup = chainedModelGroups[i];

        string endpoint = currentJointGroup->getLinkModelNames().back();
        string joint_group_name = currentJointGroup->getName();

        moveit::planning_interface::MoveGroupInterface current_move_group(joint_group_name);

        RobotStatePtr kinematic_state = current_move_group.getCurrentState();

        Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
        Eigen::MatrixXd jacobian;
        
        kinematic_state->getJacobian(currentJointGroup,kinematic_state->getLinkModel(endpoint),reference_point_position, jacobian);
        
        ROS_INFO_STREAM(currentJointGroup->getName());
        ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");
    }
}