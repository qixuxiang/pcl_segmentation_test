/**
 * @Author: Yusu Pan <yuthon>
 * @Date:   2017-06-10T16:07:14+08:00
 * @Email:  xxdsox@gmail.com
 * @Project: humanoid
 * @Filename: test_ball_detector.cpp
 * @Last modified by:   yuthon
 * @Last modified time: 2017-06-10T16:07:22+08:00
 * @Copyright: ZJUDancer
 */

#include "dvision/ball_detector.hpp"
#include "dvision/dvision.hpp"
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

using namespace dvision;

TEST(BallDetector, main)
{
    ros::NodeHandle nh;
    parameters.init(&nh);

    BallDetector ball;
    ball.Init();

    cv::VideoCapture cap(0);
    cv::Mat frame;
    cap >> frame;

    darknet::Image img(frame);
}

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "ball_detector");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
