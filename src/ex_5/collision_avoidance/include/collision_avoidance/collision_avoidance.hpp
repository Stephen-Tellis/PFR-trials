#pragma once

#include <ros/ros.h>

// msg
#include <sensor_msgs/LaserScan.h>

//srvs
#include <std_srvs/SetBool.h>

// STL
#include <string>

namespace collision_avoidance {


class CollisionAvoidance {
public:
    CollisionAvoidance(ros::NodeHandle& nodeHandle);
    virtual ~CollisionAvoidance();
    bool readParameters();
    void laserCallback(const sensor_msgs::LaserScan& message);
private:
    ros::NodeHandle nodeHandle_;
    ros::Subscriber laserSubscriber_;
    std::string subscriberTopic_;
    int queueSize_;
    ros::ServiceClient estop_srv_client_;
    std::string service_name_;

};


}