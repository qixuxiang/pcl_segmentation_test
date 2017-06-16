#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs>
#include "dtransmit/dtransmit.hpp"

using namespace dtransmit;
TEST(dtransmit, main) {
    DTransmit d("127.0.0.1");

    d.add_recv<std_msgs::UInt32>(2333, [](ActionCmd& msg) {
        ///
    });
    ActionCmd a;
    d.send<ActionCmd>(2333, a);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "localization");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}