#include "AbstractMarkerPublisher.h"

int idCounter = 0;

AbstractMarkerPublisher::AbstractMarkerPublisher(string topicName, vector<robot_model::JointModelGroup*> groups)
{
    this->groups = groups;

    ros::NodeHandle node_handle;
    this->publisher = node_handle.advertise<visualization_msgs::MarkerArray>(topicName, 0);
    
    ros::AsyncSpinner spinner(1);
    spinner.start();
}

void AbstractMarkerPublisher::startPublishing()
{
    ros::Rate r(100);

    while (ros::ok())
    {
        // TODO: fix id generation
        this->idCounter = 0;
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