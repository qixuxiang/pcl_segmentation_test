/**
 * @Author: Yusu Pan <yuthon>
 * @Date:   2017-06-04T10:29:30+08:00
 * @Email:  xxdsox@gmail.com
 * @Project: humanoid
 * @Filename: test_localization.cpp
 * @Last modified by:   yuthon
 * @Last modified time: 2017-06-04T10:29:52+08:00
 * @Copyright: ZJUDancer
 */

#include "dvision/localization.hpp"
#include <gtest/gtest.h>

using namespace dvision;
using namespace std;

TEST(Localization, locmain)
{

    Localization loc;

    //    while (ros::ok()) {
    //    }
}

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "capture");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
