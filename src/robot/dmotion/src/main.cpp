#include "dmotion/dmotion.hpp"
#include <ros/ros.h>
#include <signal.h>

using namespace dmotion;

DMotion* d = nullptr;

void signalHandler(int sig) {
    ROS_WARN("Trying to exit!");
    if (d != nullptr) {
        d->attemptShutdown();
    }
}


int
main(int argc, char** argv)
{
    ros::init(argc, argv, "dmotion_node");
    ros::NodeHandle nh;

    signal(SIGINT, signalHandler);

    d = new DMotion(&nh);
    d->spin();
    d->join();
}
