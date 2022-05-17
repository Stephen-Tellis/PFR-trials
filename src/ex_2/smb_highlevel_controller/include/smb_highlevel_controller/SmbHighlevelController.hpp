#pragma once

#include <ros/ros.h>

// msgs
#include <sensor_msgs/LaserScan.h>

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
};

} /* namespace */
