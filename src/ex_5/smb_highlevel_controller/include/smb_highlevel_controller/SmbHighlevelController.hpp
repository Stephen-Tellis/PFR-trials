#pragma once

#include <ros/ros.h>

// msgs
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>

//SRV
#include <std_srvs/SetBool.h>

// STD
#include <string>

namespace smb_highlevel_controller {

/*!
 * Class containing the Husky Highlevel Controller
 */
class SmbHighlevelController {
public:
	/*!
	 * Constructor.
	 */
	SmbHighlevelController(ros::NodeHandle& nodeHandle);

	/*!
	 * Destructor.
	 */
	virtual ~SmbHighlevelController();

	bool readParameters();
	bool estop_cb(std_srvs::SetBool::Request &req,
                        std_srvs::SetBool::Response &resp);

	bool estop_trigger(bool stop_req);

	void topicCallback(const sensor_msgs::LaserScan& message);

private:
	ros::NodeHandle nodeHandle_;
	ros::Subscriber subscriber_;
	std::string subscriberTopic_;
	ros::Publisher estop_publisher_;
	ros::Publisher cmdvel_publisher_;
	ros::Publisher marker_publisher_;
	ros::ServiceServer estop_service_;
	int queueSize_;
	double pgain_angular_;
	double pgain_linear_;
	double angle_min_;
	double angle_inc_;
	std::string service_name_;
	bool alternate_ = true;
	bool skip_control_ = false;
};

} /* namespace */
