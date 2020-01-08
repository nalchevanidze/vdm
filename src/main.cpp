#include <ros/ros.h>

#include "RobotModelTools.h"
#include "JacobianCalculator.h"
#include "RobotMarkerPublisher.h"
#include "sensor_msgs/JointState.h"


using namespace std;
using namespace robot_model;
using namespace robot_model_loader;
using namespace robot_state;
using namespace ros;


// Signature must be compatible to the respective topic
void test_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // TODO: Implement message handling
    ROS_ERROR_STREAM("msg received");
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "jacobian_calculator");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("joint_states", 1000, test_callback);
    ros::spin();

    //ros::AsyncSpinner spinner(1);
    //subscriber.startListening();
    // spinner.start();

    // RobotModelLoader robotModelLoader("robot_description");
    // RobotModelPtr kinematicModel = robotModelLoader.getModel();

    // RobotModelTools robotModelTools;
    // // JacobianCalculator jacobianCalculator;

    // const vector<JointModelGroup*>& jointModelGroups = kinematicModel->getJointModelGroups();
    // vector<JointModelGroup*> chainedModelGroups = robotModelTools.getChainModelGroups(kinematicModel);

    
    // // calculate jacobians
    // // for(int i = 0; i < chainedModelGroups.size(); i ++ )
    // // {
    // //     JointModelGroup *currentJointGroup = chainedModelGroups[i];
    // //     jacobianCalculator.calculateJacobian(currentJointGroup, true);
    // // }

    // // publish markers
    // RobotMarkerPublisher publisher("vdm_markers_velocity", chainedModelGroups);
    // publisher.startPublishing();
}
