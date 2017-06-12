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
    std::vector<darknet::bbox> ball_position;
    ball.Init();

    // get frame from camera
    cv::VideoCapture cap(0);
    cv::namedWindow("ball_detector", CV_WINDOW_NORMAL);
    cv::Mat frame, ball_img;

    while (ros::ok()) {
        cap >> frame;
        ball.GetBall(frame, ball_position);
        for (auto bbox : ball_position) {
            cv::rectangle(frame, cv::Point(bbox.m_left, bbox.m_top), cv::Point(bbox.m_right, bbox.m_bottom), cv::Scalar(0, 255, 0));
        }
        cv::imshow("ball_detector", frame);
        cv::waitKey(1);
    }
}

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "ball_detector");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
