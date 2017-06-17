#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "dtransmit/dtransmit.hpp"

// TODO(MWX): test on different machine, over wifi
using namespace dtransmit;
using namespace std;
TEST(dtransmit, main) {
//    DTransmit d("127.0.0.1");
    DTransmit d("192.168.255.255");

    int NUM = 2;

    for(int i = 0 ; ; ++i) {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "Hello dtransmit " << i;
        msg.data = ss.str();

        ROS_INFO("Sending: [%s]", msg.data.c_str());

        for(int j = 0; j < NUM; ++j) {
            d.send<std_msgs::String>(2000 + j, msg);
        }
        sleep(1);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "localization");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}