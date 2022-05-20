#include <cmath>

#include <smb_highlevel_controller/SmbHighlevelController.hpp>

// DISCLAIMER: This is a VERY bad implementation. Do not use this.
// Only done so because time of completion was the only counting factor.

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
  
  cmdvel_publisher_ = nodeHandle_.advertise<geometry_msgs::Twist> ("/cmd_vel", 10);
  estop_publisher_ = nodeHandle_.advertise<std_msgs::Bool> ("/e_stop", 10);
  marker_publisher_ = nodeHandle_.advertise<visualization_msgs::Marker> ("visualization_marker", 0);
  estop_service_ = nodeHandle_.advertiseService(service_name_, &SmbHighlevelController::estop_cb, this);
  ROS_INFO("Successfully launched node.");
}

SmbHighlevelController::~SmbHighlevelController()
{
}

bool SmbHighlevelController::estop_cb(std_srvs::SetBool::Request &req,
                                          std_srvs::SetBool::Response &resp)
{
  ROS_INFO_STREAM("Received E-stop Command");
  bool retval = SmbHighlevelController::estop_trigger(req.data);
  return retval;
}

bool SmbHighlevelController::estop_trigger(bool stop_req) {
  std_msgs::Bool msg;
  if (stop_req){
    msg.data = true;
    skip_control_ = true;
  } else {
    msg.data = false;
    skip_control_ = false;
  }
  for (int i=0; i<5; i++){
    estop_publisher_.publish(msg);
  }
  return true;
}

bool SmbHighlevelController::readParameters()
{
  if (!nodeHandle_.getParam("subscriber_topic", subscriberTopic_)) return false;
  if (!nodeHandle_.getParam("queue_size", queueSize_)) return false;
  if (!nodeHandle_.getParam("K_p_angular", pgain_angular_)) return false;
  if (!nodeHandle_.getParam("K_p_linear", pgain_linear_)) return false;
  if (!nodeHandle_.getParam("/pointcloud_to_laserscan/angle_min", angle_min_)) return false;
  if (!nodeHandle_.getParam("/pointcloud_to_laserscan/angle_increment", angle_inc_)) return false;
  if (!nodeHandle_.getParam("service_name", service_name_)) return false;
  return true;
}

void SmbHighlevelController::topicCallback(const sensor_msgs::LaserScan& message)
{
  if (!skip_control_) 
  {
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

    // Determine orientation to pillar
    double orientation_control = 0;
    orientation_control = angle_min_ + angle_inc_ * mindist_index;

    // Control 
    geometry_msgs::Twist msg;
    if (alternate_){
    msg.angular.z = orientation_control * pgain_angular_;
    alternate_ = false;
    } else {
    msg.linear.x = minVal * pgain_linear_;
    alternate_ = true;
    }
    // ROS_INFO_STREAM("Sending command: " << "linear=" << msg.linear.x << " angular=" << msg.angular.z);
    cmdvel_publisher_.publish(msg);

    // Visualization marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = "rslidar";
    marker.header.stamp = ros::Time();
    marker.ns = "";
    marker.id = 5555;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = minVal;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker_publisher_.publish( marker );
  } else {
    geometry_msgs::Twist msg;
    ROS_INFO_STREAM("Sending command: " << "linear=" << msg.linear.x << " angular=" << msg.angular.z);
    cmdvel_publisher_.publish(msg);
  }
}

} /* namespace */
