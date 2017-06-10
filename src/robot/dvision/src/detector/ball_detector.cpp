/**
 * @Author: Yusu Pan <yuthon>
 * @Date:   2017-06-10T10:43:26+08:00
 * @Email:  xxdsox@gmail.com
 * @Project: humanoid
 * @Filename: ball_detector.cpp
 * @Last modified by:   yuthon
 * @Last modified time: 2017-06-10T10:43:43+08:00
 * @Copyright: ZJUDancer
 */

#include "dvision/ball_detector.hpp"
#include "dvision/parameters.hpp"

namespace dvision {
BallDetector::BallDetector()
{
}

bool
BallDetector::Init()
{
    ROS_INFO("BallDetector Init() started");
    return true;
}

BallDetector::~BallDetector()
{
}

bool
BallDetector::GetBall()
{
    return true;
}

} // namespace dvision
