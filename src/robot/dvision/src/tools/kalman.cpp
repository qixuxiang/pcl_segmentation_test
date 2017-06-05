/**
 * @Author: Yusu Pan <yuthon>
 * @Date:   2017-06-05T19:24:53+08:00
 * @Email:  xxdsox@gmail.com
 * @Project: humanoid
 * @Filename: kalman.cpp
 * @Last modified by:   yuthon
 * @Last modified time: 2017-06-05T19:24:56+08:00
 * @Copyright: ZJUDancer
 */

#include "dvision/kalman.hpp"

namespace dvision {
KalmanFilterC::KalmanFilterC(const cv::Point2f& pt)
{
    kalman = new cv::KalmanFilter(6, 2, 0);
    cv::Mat processNoise(6, 1, CV_32F);

    kalman->statePre.at<float>(0) = pt.x;
    kalman->statePre.at<float>(1) = pt.y;
    kalman->statePre.at<float>(2) = 0;
    kalman->statePre.at<float>(3) = 0;
    kalman->statePre.at<float>(4) = 0;
    kalman->statePre.at<float>(5) = 0;
    kalman->transitionMatrix = (cv::Mat_<float>(6, 6) << 1, 0, 1, 0, 0.5, 0, 0, 1, 0, 1, 0, 0.5, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1);
    kalman->measurementMatrix = (cv::Mat_<float>(2, 6) << 1, 0, 1, 0, 0.5, 0, 0, 1, 0, 1, 0, 0.5);
    cv::setIdentity(kalman->measurementMatrix);
    cv::setIdentity(kalman->processNoiseCov, cv::Scalar::all(1e-4));
    cv::setIdentity(kalman->measurementNoiseCov, cv::Scalar::all(1e-1));
    cv::setIdentity(kalman->errorCovPost, cv::Scalar::all(.1));
}

KalmanFilterC::~KalmanFilterC()
{
    delete kalman;
}

cv::Point2f
KalmanFilterC::GetPrediction()
{
    cv::Mat prediction = kalman->predict();
    return cv::Point2f(prediction.at<float>(0), prediction.at<float>(1));
}

cv::Point2f
KalmanFilterC::Update(const cv::Point2f& p)
{
    cv::Mat measurement(2, 1, CV_32FC1);

    measurement.at<float>(0) = p.x;
    measurement.at<float>(1) = p.y;

    cv::Mat estimated = kalman->correct(measurement);
    return cv::Point2f(estimated.at<float>(0), estimated.at<float>(1));
}

} // namespace dvision
