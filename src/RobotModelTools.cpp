#include <iostream>
#include <string>
#include "RobotModelTools.h"

vector<robot_model::JointModelGroup*> 
RobotModelTools::getChainModelGroups(robot_model::RobotModelPtr model)
{
    const vector<robot_model::JointModelGroup*> &jointModelGroups =
        model->getJointModelGroups();
    vector<robot_model::JointModelGroup*> chainModelGroups;

    for (int i = 0; i < jointModelGroups.size(); i++)
    {
        robot_model::JointModelGroup *current = jointModelGroups[i];
        if (current->isChain())
        {
            chainModelGroups.push_back(current);
            ROS_INFO("Found chain-model-group: %s", current->getName().c_str());
        }
    }

    return chainModelGroups;
}

vector<string> 
RobotModelTools::allJointNames(vector<robot_model::JointModelGroup*> groups)
{
    vector<string> jointNames;

    for (int i = 0; i < groups.size(); i++)
    {
        robot_model::JointModelGroup *currentJointGroup = groups[i];
        vector<string> jointNames = currentJointGroup->getLinkModelNames();
        
        for (int j = 0; j < jointNames.size(); j++) 
        {
            jointNames.push_back(jointNames[i]);
        }
    }

    return jointNames;
}