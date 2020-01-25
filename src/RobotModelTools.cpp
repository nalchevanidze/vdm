#include <iostream>
#include <string>
#include "RobotModelTools.h"

vector<string> 
RobotModelTools::getAllJointNames(robot_model::RobotModelPtr model)
{
    const vector<robot_model::JointModelGroup*> &jointModelGroups = model->getJointModelGroups();
    vector<string> result;

    for (int i = 0; i < jointModelGroups.size(); i++)
    {
        robot_model::JointModelGroup *current = jointModelGroups[i];

        if (current->isChain())
        {
            ROS_INFO("Found chain-model-group: %s", current->getName().c_str());
            vector<string> jointNames = current->getLinkModelNames();

            // concatenate all joints Names
            result.insert(result.end(), jointNames.begin(), jointNames.end());
        }
    }

    ROS_INFO_STREAM("listed joint count: " << result.size());
    return result;
}