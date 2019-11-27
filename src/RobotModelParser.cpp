#include "RobotModelParser.h"

robot_model::RobotModelPtr RobotModelParser::getRobotModelLoader()
{
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    return kinematic_model;
}

const std::vector<std::string>& RobotModelParser::getJointModelNames(robot_model::RobotModelPtr robot_model)
{
    return robot_model->getJointModelNames();
}