#include <gtest/gtest.h>
#include <ros/ros.h>
#include "dtransmit/dtransmit.hpp"

using namespace dtransmit;
TEST(dtransmit, main) {
    DTransmit d("192.168.255.255");

    d.add_recv<ActionCmd>(2333, [](ActionCmd& msg) {
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