#include "VelocityCalculator.h"

tf2_ros::Buffer *tfBuffer = new tf2_ros::Buffer();
tf2_ros::TransformListener *tfListener = new tf2_ros::TransformListener(*tfBuffer);

chrono::high_resolution_clock::time_point lastTimePoint = chrono::high_resolution_clock::now();
vector<double> lastCoordinates = { 0.0, 0.0, 0.0 };


double VelocityCalculator::calculateVelocity(
    chrono::high_resolution_clock::time_point t1, 
    vector<double> pos1, 
    chrono::high_resolution_clock::time_point t2, 
    vector<double> pos2
)
{
    // Hardcoded Euclidian distance between two 3-dimensional vectors
    double distance = sqrt(pow(pos1[0] - pos2[0], 2.0) + pow(pos1[1] - pos2[1], 2.0) + pow(pos1[2] - pos2[2], 2.0));
    chrono::duration<double, std::milli> timeDiff = t1 - t2;

    // v = s / t
    return (distance / (timeDiff.count()));
}

double VelocityCalculator::velocityByJointName(string name) 
{
   chrono::high_resolution_clock::time_point currentTimePoint = chrono::high_resolution_clock::now();

    
    geometry_msgs::TransformStamped transformStamped;
    try
    {
       // transformStamped = tfBuffer.lookupTransform("base_link", name, ros::Time(0));
    }
    catch (tf2::TransformException &ex) 
    {
        //ROS_WARN_THROTTLE(1.0, "%s",ex.what());
        return 0.0;
    }


    double x = transformStamped.transform.translation.x;
    double y = transformStamped.transform.translation.y;
    double z = transformStamped.transform.translation.z;

    ROS_ERROR_STREAM("Coordinates: " << "x: " << x << ", y: " << y << ", z: " << z);
    vector<double> currentCoordinates = { x, y, z };

    
    double velocity = calculateVelocity(currentTimePoint, currentCoordinates, lastTimePoint, lastCoordinates);
    ROS_ERROR_STREAM("Velocity: " << velocity);


    lastTimePoint = currentTimePoint;
    lastCoordinates = currentCoordinates;

    return velocity;
}