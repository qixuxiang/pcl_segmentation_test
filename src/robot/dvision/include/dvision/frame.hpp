// Created on: May 20, 2017
//     Author: Wenxing Mei <mwx36mwx@gmail.com>

#pragma once
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <string>

namespace dvision {
class Frame
{
  public:
    inline explicit Frame(uint8_t* yuv, int width, int height)
      : m_yuv(yuv)
      , m_width(width)
      , m_height(height)
      , m_converted(false)
    {
        m_timeStamp = ros::Time::now();
    }

    inline Frame(std::string filepath)
    {
        m_bgr = cv::imread(filepath);
        m_width = m_bgr.size().width;
        m_height = m_bgr.size().height;
        m_converted = true;
    }

    inline Frame()
    {
    }

    inline explicit Frame(cv::Mat& mat, int width, int height)
      : m_bgr(mat)
      , m_width(width)
      , m_height(height)
      , m_converted(true)
    {
        m_timeStamp = ros::Time::now();
    }

    inline ~Frame()
    {
    }

    inline cv::Mat getRGB()
    {
        cvt();
        cv::Mat rgb;
        cv::cvtColor(m_bgr, rgb, CV_BGR2RGB);
        return rgb;
    }

    inline cv::Mat& getBGR()
    {
        cvt();
        return m_bgr;
    }

    inline cv::Mat getBGR_raw()
    {
        cvt();
        return m_bgr;
    }

    inline cv::Mat& getHSV()
    {
        cvt();
        return m_hsv;
    }

    inline void cvt()
    {
        if (m_converted)
            return;
        cv::Mat yuvMat(m_height, m_width, CV_8UC2, m_yuv);
        cv::cvtColor(yuvMat, m_bgr, CV_YUV2BGR_YUYV);
        cv::cvtColor(m_bgr, m_hsv, CV_BGR2HSV);
        m_converted = true;
    }

    void show();

    void save(std::string path);

  private:
    uint8_t* m_yuv; // raw yuv image
    cv::Mat m_bgr;
    cv::Mat m_hsv;

    ros::Time m_timeStamp;
    int m_width;
    int m_height;
    bool m_converted;
};
} // namespace dvision
