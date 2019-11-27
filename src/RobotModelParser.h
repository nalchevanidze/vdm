#ifndef VDM_ROBOTMODELPARSER_H
#define VDM_ROBOTMODELPARSER_H


#include <ros/ros.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

class RobotModelParser {

public:
    robot_model::RobotModelPtr getRobotModelLoader();
    const std::vector<std::string>& getJointModelNames(robot_model::RobotModelPtr robot_model);
};


#endif //VDM_ROBOTMODELPARSER_H
