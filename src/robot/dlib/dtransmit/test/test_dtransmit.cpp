#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "dtransmit/dtransmit.hpp"

using namespace dtransmit;
TEST(dtransmit, main) {
    DTransmit d("127.0.0.1");

    d.add_recv<std_msgs::String>(2333, [](std_msgs::String& msg) {
        ROS_INFO("I heard: [%s]", msg.data.c_str());
    });


    for(int i = 0 ; ; ++i) {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "Hello dtransmit" << i;
        msg.data = ss.str();

       ROS_INFO("Sending: [%s]", msg.data.c_str());

        d.send<std_msgs::String>(2333, msg);
        sleep(1);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "localization");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}