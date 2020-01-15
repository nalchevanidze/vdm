#include <ros/ros.h>

#include "RobotModelTools.h"
#include "JacobianCalculator.h"
#include "RobotMarkerPublisher.h"
#include "sensor_msgs/JointState.h"

#include <moveit/move_group_interface/move_group_interface.h>

using namespace std;
using namespace robot_model;
using namespace robot_model_loader;
using namespace robot_state;
using namespace ros;


Publisher pub;
int cnt = 1000;


double calculateVelocity(time t1, double pos1, time t2, double pos2)
{
    return 1.0d;
}


// Signature must be compatible to the respective topic
void test_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    //time timestamp = msg->header.stamp;
    //ROS_INFO_STREAM("Timestamp: " << timestamp);

    V_string name = msg->name;
    vector<double> velocity = msg->velocity;
    vector<double> position = msg->position;

    ROS_INFO_STREAM("Name size: " << name.size());
    ROS_INFO_STREAM("Velocity size: " << velocity.size());
    ROS_INFO_STREAM("Position size: " << position.size());

    for (int i = 0; i < msg->position.size(); i++) {
        ROS_INFO_STREAM("Current name: " << name[i]);
        ROS_INFO_STREAM("Current pos: " << position[i]);
        ROS_INFO_STREAM("Current position: " << position[i]);


        // Marker only as a proof of concept - use outsourced method for creation instead

        RobotModelLoader robotModelLoader("robot_description");
        RobotModelPtr kinematicModel = robotModelLoader.getModel();
        auto jointModel = kinematicModel->getJointModel(name[i]);
        auto jointChildLinkModel = jointModel->getChildLinkModel();
        string linkName = jointChildLinkModel->getName();

        
        visualization_msgs::MarkerArray markerArray;
        visualization_msgs::Marker marker;
        marker.header.frame_id = linkName;
        marker.header.stamp = ros::Time();
        marker.ns = "stats";
        marker.id = ++cnt;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.03;
        marker.scale.y = 0.03;
        marker.scale.z = 0.03;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        markerArray.markers.push_back(marker);

        pub.publish(markerArray);
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "jacobian_calculator");

    ros::NodeHandle n;

    pub = n.advertise<visualization_msgs::MarkerArray>("vdm_test_arrays", 0);

    ros::Subscriber sub = n.subscribe("joint_states", 1000, test_callback);
    ros::spin();


    // TODO: Dead code, should be deleted in the future (AbstractMArkerPublisher too)!

    //ros::AsyncSpinner spinner(1);
    //subscriber.startListening();
    // spinner.start();

    // RobotModelLoader robotModelLoader("robot_description");
    // RobotModelPtr kinematicModel = robotModelLoader.getModel();

    // RobotModelTools robotModelTools;
    // JacobianCalculator jacobianCalculator;

    // const vector<JointModelGroup*>& jointModelGroups = kinematicModel->getJointModelGroups();
    // vector<JointModelGroup*> chainedModelGroups = robotModelTools.getChainModelGroups(kinematicModel);

    // publish markers
    // RobotMarkerPublisher publisher("vdm_markers_velocity", chainedModelGroups);
    // publisher.startPublishing();
}
