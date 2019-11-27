#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

using namespace robot_model;

int main(int argc, char** argv) {
    ros::init(argc, argv, "jacobian_calculator");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");


    moveit::planning_interface::MoveGroupInterface move_group_left_leg("LeftLeg");
    
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    
    ROS_ERROR("Model frame: %s", kinematic_model->getModelFrame().c_str());

    robot_state::RobotStatePtr kinematic_state = move_group_left_leg.getCurrentState();
    
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("LeftLeg");

    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    Eigen::MatrixXd jacobian;

    const std::vector<JointModel*>& joint_models = kinematic_model->getJointModels();

    
    for (std::size_t i = 0; i < joint_models.size(); ++i)
    {
        const JointModel* jointModel = joint_models[i];



        ROS_INFO("Joint: %s", jointModel->getName().c_str());
    }

    kinematic_state->getJacobian(joint_model_group,
                               kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                               reference_point_position, jacobian);
    
    ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");
}