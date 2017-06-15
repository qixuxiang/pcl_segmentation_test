// Created on: May 20, 2017
//     Author: Wenxing Mei <mwx36mwx@gmail.com>

#include "dvision/frame.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

namespace dvision {
void
Frame::save(std::string path)
{
    cvt();
    std::string filename = path + std::to_string(m_timeStamp.toNSec()) + ".png";
    cv::imwrite(filename, m_bgr);

    std::cout << "Saved as " << filename << std::endl;
}

void
Frame::show()
{
    cvt();
    cv::namedWindow("camera", CV_WINDOW_NORMAL);

    cv::imshow("camera", m_bgr);
}
} // namespace dvision
