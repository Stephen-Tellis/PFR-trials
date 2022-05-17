#include <smb_highlevel_controller/SmbHighlevelController.hpp>

namespace smb_highlevel_controller {

SmbHighlevelController::SmbHighlevelController(ros::NodeHandle& nodeHandle) :
  nodeHandle_(nodeHandle)
{
    if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  subscriber_ = nodeHandle_.subscribe(subscriberTopic_, queueSize_,
                                      &SmbHighlevelController::topicCallback, this);

  ROS_INFO("Successfully launched node.");
}

SmbHighlevelController::~SmbHighlevelController()
{
}

bool SmbHighlevelController::readParameters()
{
  if (!nodeHandle_.getParam("subscriber_topic", subscriberTopic_)) return false;
  if (!nodeHandle_.getParam("queue_size", queueSize_)) return false;
  return true;
}

void SmbHighlevelController::topicCallback(const sensor_msgs::LaserScan& message)
{

  int size = message.ranges.size();
  int minIndex = 0;
  int maxIndex = size; 
  int closestIndex = -1;
  double minVal = 999;

  for (int i = minIndex; i < maxIndex; i++)
  {
      if ((message.ranges[i] <= minVal) && (message.ranges[i] >= message.range_min) && (message.ranges[i] <= message.range_max))
      {
          minVal = message.ranges[i];
      }
  }
  ROS_INFO_STREAM("Nearest point is:" << minVal); 
}

} /* namespace */
