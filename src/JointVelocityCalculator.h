#ifndef JOINT_VELOCITY_CALCULATOR_H
#define JOINT_VELOCITY_CALCULATOR_H


#include <ctime>
#include <ratio>
#include <chrono>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>


using namespace std;


typedef chrono::high_resolution_clock::time_point timepoint_t;


/// A listener that can be attached to a tf2-Buffer in order to calculate each
/// joint's velocity from the incoming positions.
class JointVelocityCalculator 
{
public:
    /// Constructs a JointVelocityCalculator.
    ///
    /// In order for the calculator to work, a tf-buffer, a complete list of
    /// available joints as well as a callback function need to be passed in.
    ///
    /// The callback function takes the name of the joint which velocity has
    /// been calculated as the first and the actual velocity as the second
    /// argument.
    JointVelocityCalculator(
        ros::NodeHandle *nodeHandle,
        tf2_ros::Buffer *tfBuffer,
        vector<string> *jointNames,
        void (*onVelocityCalculated)(string, double)
    );

    /// Tells the calculator to start listening for incoming positions.
    void startListening();

private:
    /// The applications main node-handle
    ros::NodeHandle *nodeHandle;

    /// The tf-buffer to read the joint-positions from.
    tf2_ros::Buffer *tfBuffer;

    /// The list of all joint-names passed in via the constructor.
    vector<string> *jointNames;

    /// The callback function to call when a joint's velocity has been 
    /// calculated.
    void (*onVelocityCalculated)(string, double);


    /// Converts a TransformStamped object into a three-dimenional vector
    /// containing the respective x, y and z values.
    vector<double> transformStampedToPos(geometry_msgs::TransformStamped msg);

    /// Calculates a joint's velocity from two given time points and position 
    /// vectors.
    double calculateVelocity(
        timepoint_t t1, 
        vector<double> pos1, 
        timepoint_t t2, 
        vector<double> pos2
    );
};

#endif