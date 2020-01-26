#ifndef JOINT_VELOCITY_CALCULATOR_H
#define JOINT_VELOCITY_CALCULATOR_H


#include <ctime>
#include <ratio>
#include <chrono>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>


using namespace std;


typedef chrono::high_resolution_clock::time_point timepoint_t;

typedef vector<double> coordinates_t;


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


    /// Retrieves the current joint's position and calculates it's velocity
    /// using the values from the respective buffers.
    ///
    /// The time-point-buffer is used to hold the time-point from the last 
    /// calculation, which is necessary to be able to calculate the velocity.
    /// The coordinates-buffer works the same way but for the last coordinates.
    ///
    /// Important: Both buffers need to hold a valid value at all times!
    /// Initialize them with default values if necessary.
    double getVelocityForJoint(
        string jointName, 
        timepoint_t *timePointBuffer,
        coordinates_t *coordinatesBuffer
    );


    /// Converts a TransformStamped object into a three-dimenional vector
    /// containing the respective x, y and z values.
    coordinates_t transformStampedToCoordinates(
        geometry_msgs::TransformStamped msg
    );

    /// Calculates a joint's velocity from two given time points and position 
    /// vectors.
    double calculateVelocity(
        timepoint_t t1, 
        coordinates_t pos1, 
        timepoint_t t2, 
        coordinates_t pos2
    );
};

#endif