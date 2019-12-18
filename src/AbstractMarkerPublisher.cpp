#include "AbstractMarkerPublisher.h"

int idCounter = 0;

AbstractMarkerPublisher::AbstractMarkerPublisher(vector<robot_model::JointModelGroup*> groups)
{
    this->groups = groups;

    ros::NodeHandle node_handle;
    this->publisher = node_handle.advertise<visualization_msgs::MarkerArray>(getPublisherTopicName(), 0);
    
    ros::AsyncSpinner spinner(1);
    spinner.start();
}

void AbstractMarkerPublisher::startPublishing()
{
    ros::Rate r(100);

    while (ros::ok())
    {
        for (int i = 0; i < groups.size(); i++)
        {
            robot_model::JointModelGroup *currentJointGroup = groups[i];
            vector<string> jointNames = currentJointGroup->getLinkModelNames();
            
            for (int j = 0; j < jointNames.size(); j++) 
            {
                string name = jointNames[j]; 

                publisher.publish(createMarkersForFrame(name));
            }
        }
        r.sleep();
    }
}