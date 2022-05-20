#include<ros/ros.h>
#include "collision_avoidance/collision_avoidance.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "collision_avoidance_client");
    ros::NodeHandle nodeHandle("~");

    collision_avoidance::CollisionAvoidance collision_avoidance(nodeHandle);

    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}