/**
 * @Author: Yusu Pan <yuthon>
 * @Date:   2017-06-05T19:25:53+08:00
 * @Email:  xxdsox@gmail.com
 * @Project: humanoid
 * @Filename: kalman.hpp
 * @Last modified by:   yuthon
 * @Last modified time: 2017-06-05T19:25:54+08:00
 * @Copyright: ZJUDancer
 */

#pragma once
#include "opencv2/opencv.hpp"
#include <vector>

namespace dvision {
class KalmanFilterC
{
  public:
    explicit KalmanFilterC(const cv::Point2f& p);
    ~KalmanFilterC();

    cv::Point2f GetPrediction();
    cv::Point2f Update(const cv::Point2f& p);

    cv::KalmanFilter* kalman_;

  private:
};
} // namespace dvision
