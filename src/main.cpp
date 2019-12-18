#include <ros/ros.h>

#include "RobotModelTools.h"
#include "JacobianCalculator.h"
#include "RobotMarkerPublisher.h"


using namespace std;
using namespace robot_model;
using namespace robot_model_loader;
using namespace robot_state;
using namespace ros;


int main(int argc, char** argv) {
    ros::init(argc, argv, "jacobian_calculator");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    RobotModelLoader robotModelLoader("robot_description");
    RobotModelPtr kinematicModel = robotModelLoader.getModel();

    RobotModelTools robotModelTools;
    // JacobianCalculator jacobianCalculator;

    const vector<JointModelGroup*>& jointModelGroups = kinematicModel->getJointModelGroups();
    vector<JointModelGroup*> chainedModelGroups = robotModelTools.getChainModelGroups(kinematicModel);

    
    // calculate jacobians
    // for(int i = 0; i < chainedModelGroups.size(); i ++ )
    // {
    //     JointModelGroup *currentJointGroup = chainedModelGroups[i];
    //     jacobianCalculator.calculateJacobian(currentJointGroup, true);
    // }

    // publish markers
    RobotMarkerPublisher publisher("vdm_markers_velocity", chainedModelGroups);
    publisher.startPublishing();
}
