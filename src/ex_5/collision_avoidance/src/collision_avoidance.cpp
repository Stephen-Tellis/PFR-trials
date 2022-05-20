#include "collision_avoidance/collision_avoidance.hpp"

namespace collision_avoidance {

CollisionAvoidance::CollisionAvoidance(ros::NodeHandle& nodeHandle) :
    nodeHandle_(nodeHandle)
{
    if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
    }
    laserSubscriber_ = nodeHandle_.subscribe(subscriberTopic_, queueSize_,
                                            &CollisionAvoidance::laserCallback, this);
    estop_srv_client_ = nodeHandle_.serviceClient<std_srvs::SetBool>("/smb_highlevel_controller/estop");
    ROS_INFO("Successfully launched node.");
}

CollisionAvoidance::~CollisionAvoidance()
{
}

bool CollisionAvoidance::readParameters()
{
  if (!nodeHandle_.getParam("/smb_highlevel_controller/subscriber_topic", subscriberTopic_)) return false;
  if (!nodeHandle_.getParam("/smb_highlevel_controller/queue_size", queueSize_)) return false;
  if (!nodeHandle_.getParam("/smb_highlevel_controller/service_name", service_name_)) return false;
  return true;
}

void CollisionAvoidance::laserCallback(const sensor_msgs::LaserScan& message){
    int size = message.ranges.size();
    int minIndex = 0;
    int maxIndex = size; 
    int closestIndex = -1;
    double minVal = 999;
    int mindist_index = 0;

    // Determine distance to pillar
    for (int i = minIndex; i < maxIndex; i++)
    {
        if ((message.ranges[i] <= minVal) && (message.ranges[i] >= message.range_min) && (message.ranges[i] <= message.range_max))
        {
            minVal = message.ranges[i];
            mindist_index = i;
        }
    }
    ROS_INFO_STREAM("minval: " << minVal);
    if (minVal<2){
        ROS_INFO_STREAM("TOO CLOSE STOPPING");
        std_srvs::SetBool msgg;
        msgg.request.data = true;
        if (estop_srv_client_.call(msgg)){
            ROS_INFO_STREAM("Conveyed stop request");
        } else {
            ROS_INFO_STREAM("Stop request not successful");
        }
    }
}

}