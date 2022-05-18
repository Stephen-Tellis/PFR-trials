#pragma once

#include <ros/ros.h>

// msgs
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>

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

	void topicCallback(const sensor_msgs::LaserScan& message);

private:
	ros::NodeHandle nodeHandle_;
	ros::Subscriber subscriber_;
	std::string subscriberTopic_;
	int queueSize_;
	double pgain_angular_;
	double pgain_linear_;
	double angle_min_;
	double angle_inc_;
	bool alternate_ = true;
	ros::Publisher publisher_;
	ros::Publisher publisher_marker_;
};

} /* namespace */
