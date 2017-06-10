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
    // init node handle and params
    ros::NodeHandle nh;
    parameters.init(&nh);

    // init ball detector
    BallDetector ball;
    std::vector<cv::Point2d> ball_position;
    ball.Init();

    // get frame from camera
    cv::VideoCapture cap(0);
    cv::Mat cv_frame;

    while (ros::ok()) {
        cap >> cv_frame;
        ball.GetBall(cv_frame, ball_position);
    }
}

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "ball_detector");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
