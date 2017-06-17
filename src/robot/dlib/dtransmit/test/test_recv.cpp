#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "dtransmit/dtransmit.hpp"

using namespace dtransmit;
using namespace std;
TEST(dtransmit, main) {
    DTransmit d("192.168.255.255");
    int NUM = 2;

    for(int i = 0; i < NUM; ++i) {
        d.add_recv<std_msgs::String>(2000 + i, [=](std_msgs::String& msg) {
            ROS_INFO("%d heard: [%s]", 2000 + i, msg.data.c_str());
        });
    }

    while(true) {
        sleep(1);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "localization");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}