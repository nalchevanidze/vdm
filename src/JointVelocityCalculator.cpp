#include "JointVelocityCalculator.h"


JointVelocityCalculator::JointVelocityCalculator(
    ros::NodeHandle *nodeHandle,
    tf2_ros::Buffer *tfBuffer,
    vector<string> *jointNames,
    void (*onVelocityCalculated)(string, double)

) : nodeHandle(nodeHandle),
    tfBuffer(tfBuffer),
    jointNames(jointNames),
    onVelocityCalculated(onVelocityCalculated)
{
    // Nothing to do here
}


void JointVelocityCalculator::startListening()
{
    const int jointNamesSize = this->jointNames->size();

    // Store the previously received values for each joint in a vector
    //
    // The joint's value is referenced via it's index in the list of all joints
    // that has been provided via the constructor.
    //
    // Since the previous values don't exist on the first iteration both vectors
    // are initialized with default values.
    vector<timepoint_t> lastTimePoints(
        jointNamesSize, chrono::high_resolution_clock::now());
    vector<coordinates_t> lastCoordinates(jointNamesSize, { 0.0, 0.0, 0.0 });
   
    
    ros::Rate rate(10.0);
    while (this->nodeHandle->ok())
    {
        for (int i = 0; i < this->jointNames->size(); i++) 
        {
            const string currentJointName = (*this->jointNames)[i];


            double velocity = getVelocityForJoint(
                currentJointName,
                &lastTimePoints[i],
                &lastCoordinates[i]
            );


            // invoke callback function

            this->onVelocityCalculated(currentJointName, velocity);
        }
        rate.sleep();
    }
}

double JointVelocityCalculator::getVelocityForJoint(
    string jointName, 
    timepoint_t *timePointBuffer,
    coordinates_t *coordinatesBuffer
)
{
    // Receive current time-point and joint-coordinates

    geometry_msgs::TransformStamped transformStamped;
    try
    {
        transformStamped = this->tfBuffer->lookupTransform(
            "base_link", 
            jointName, 
            ros::Time(0)
        );
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN(
            "tf2: unable to get transform of joint '%s':\n%s", 
            jointName.c_str(), 
            ex.what()
        );
        return 0.0;
    }

    timepoint_t currentTimePoint = chrono::high_resolution_clock::now();
    coordinates_t currentCoordinates = 
        transformStampedToCoordinates(transformStamped);


    // Calculate velocity

    double velocity = calculateVelocity(
        currentTimePoint, 
        currentCoordinates, 
        *timePointBuffer, 
        *coordinatesBuffer
    );


    // Log current values:

    if (velocity != 0.0) {
        ROS_WARN_STREAM("Joint: " << jointName);
        ROS_INFO_STREAM(
            "Coordinates: { x: " << currentCoordinates[0] <<
            ", y: " << currentCoordinates[1] <<
            ", z: " << currentCoordinates[2] << " }"
        );
        ROS_INFO_STREAM("Velocity: " << velocity << " m/s\n");
    }


    // Update previous values

    *timePointBuffer = currentTimePoint;
    *coordinatesBuffer = currentCoordinates;


    // Return current velocity
    return velocity;
}


double JointVelocityCalculator::calculateVelocity(
    timepoint_t t1, 
    vector<double> pos1, 
    timepoint_t t2, 
    vector<double> pos2
)
{
    // Hardcoded Euclidian distance between two 3-dimensional vectors
    double distance = sqrt(pow(pos1[0] - pos2[0], 2.0) + pow(pos1[1] - pos2[1], 2.0) + pow(pos1[2] - pos2[2], 2.0));
    chrono::duration<double, std::milli> timeDiff = t1 - t2;

    // v = s / t
    return (distance / (timeDiff.count())) * 1000;
}


coordinates_t JointVelocityCalculator::transformStampedToCoordinates(
    geometry_msgs::TransformStamped msg
)
{
    double x = msg.transform.translation.x;
    double y = msg.transform.translation.y;
    double z = msg.transform.translation.z;
    return { x, y, z };
}