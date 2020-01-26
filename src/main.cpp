#include <ctime>
#include <ratio>
#include <chrono>

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include "JointVelocityCalculator.h"
#include "RobotModelTools.h"
#include "RobotMarkerGenerator.h"
#include "sensor_msgs/JointState.h"

using namespace std;
using namespace robot_model;
using namespace robot_model_loader;
using namespace robot_state;
using namespace ros;


int main(int argc, char** argv) {
    ros::init(argc, argv, "jacobian_calculator");
    ros::NodeHandle n;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // list all joints
    RobotModelLoader robotModelLoader("robot_description");
    RobotModelPtr kinematicModel = robotModelLoader.getModel();
    RobotModelTools robotModelTools;
    vector<string> jointNames = robotModelTools.getAllJointNames(kinematicModel);

    // publish velocities for all joint
    auto publisher = n.advertise<visualization_msgs::MarkerArray>("/vdm_markers", 0);
    RobotMarkerGenerator markerGenerator = RobotMarkerGenerator();

    JointVelocityCalculator vcalc(&n, &tfBuffer, &jointNames, NULL);
    vcalc.startListening();
}
