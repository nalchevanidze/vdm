#include <iostream>
#include <string>
#include "RobotModelTools.h"

vector<robot_model::JointModelGroup*> 
*RobotModelTools::getChainModelGroups(robot_model::RobotModelPtr model)
{
    const vector<robot_model::JointModelGroup*> &jointModelGroups =
        model->getJointModelGroups();
    vector<robot_model::JointModelGroup*> *chainModelGroups =
        new vector<robot_model::JointModelGroup*>();

    for (int i = 0; i < jointModelGroups.size(); i++)
    {
        robot_model::JointModelGroup *current = jointModelGroups[i];
        if (current->isChain())
        {
            chainModelGroups->push_back(current);
            ROS_INFO("Found chain-model-group: %s", current->getName());
        }
    }

    return chainModelGroups;
}