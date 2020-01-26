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
    // init velocities,cordinates for each joint
    vector<chrono::high_resolution_clock::time_point> lastTimePoints; // chrono::high_resolution_clock::now();
    vector<vector<double>> lastCoordinates; // { 0.0, 0.0, 0.0 };
    // fill initialize times and positions
    // TODO: Use vector constructor to initialize
    for (int j = 0; j < this->jointNames->size(); j++) 
    {
        lastTimePoints.push_back(chrono::high_resolution_clock::now());
        vector<double> dummyCoordninates = { 0.0, 0.0, 0.0 };
        lastCoordinates.push_back(dummyCoordninates);
    }
   
    // // publish velocities for all joint
    // auto publisher = n.advertise<visualization_msgs::MarkerArray>("/vdm_markers", 0);
    // RobotMarkerGenerator markerGenerator = RobotMarkerGenerator();
    
    ros::Rate rate(10.0);
    while (this->nodeHandle->ok())
    {
        // update Positions
        for (int i = 0; i < this->jointNames->size(); i++) 
        {
            // Vector-pointer needs to be dereferenced:
            string name = (*this->jointNames)[i]; 
            ROS_WARN_STREAM("Name: " << name);

            timepoint_t currentTimePoint = chrono::high_resolution_clock::now();

            geometry_msgs::TransformStamped transformStamped;
            try
            {
                transformStamped = this->tfBuffer->lookupTransform("base_link", name, ros::Time(0));
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s",ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }

            vector<double> currentCoordinates = transformStampedToPos(transformStamped);    
            double velocity = calculateVelocity(currentTimePoint, currentCoordinates, lastTimePoints[i], lastCoordinates[i]);
            ROS_INFO_STREAM("Velocity: " << velocity);

            lastTimePoints[i] = currentTimePoint;
            lastCoordinates[i] = currentCoordinates;

            // publisher.publish(markerGenerator.createVelocityMarkers(name, velocity));
        }
        rate.sleep();
    }
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
    return (distance / (timeDiff.count()));
}


vector<double> JointVelocityCalculator::transformStampedToPos(
    geometry_msgs::TransformStamped msg
)
{
    double x = msg.transform.translation.x;
    double y = msg.transform.translation.y;
    double z = msg.transform.translation.z;
    ROS_INFO_STREAM("Coordinates: {" << "x: " << x << ", y: " << y << ", z: " << z << "}");
    return { x, y, z };
}