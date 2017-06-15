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
#include <fstream>

namespace dvision {
BallDetector::BallDetector()
  : net_(NULL)
{
}

bool
BallDetector::Init()
{
    // parse label list
    std::cout << parameters.ball.label_file << std::endl;
    if (!std::ifstream(parameters.ball.label_file)) {
        ROS_ERROR("darknet label file doesn't exist in %s!", parameters.ball.label_file.c_str());
        return false;
    }
    label_list_ = darknet::get_labels(parameters.ball.label_file);

    // parse net cfg and setup network
    if (!std::ifstream(parameters.ball.net_cfg)) {
        ROS_ERROR("darknet network cfg doesn't exist in %s!", parameters.ball.net_cfg.c_str());
        return false;
    }

    // setup network
    net_ = &(darknet::parse_network_cfg(parameters.ball.net_cfg));
    darknet::params p = net_->get_params();
    ROS_INFO("network setup: num_layers = %d, batch = %d\n", net_->num_layers(), p.batch);

    // load pretrained weights
    if (!std::ifstream(parameters.ball.weight_file)) {
        ROS_ERROR("darknet weigth file doesn't exist in %s!", parameters.ball.weight_file.c_str());
        return false;
    }
    darknet::load_weights(net_, parameters.ball.weight_file);

    // set batch to 1 for network inference
    net_->set_network_batch(1);

    ROS_INFO("BallDetector Init() finished");
    return true;
}

BallDetector::~BallDetector()
{
    delete net_;
}

bool
BallDetector::GetBall(cv::Mat& frame, VisionShareData& m_data, Projection& m_projection)
{
    darknet::Image raw_img(frame);
    darknet::params p = net_->get_params();
    ROS_DEBUG("darknet resize image to %d x %d x %d", p.h, p.w, p.c);
    raw_img.resize_neo(p.h, p.w, p.c);

    std::vector<darknet::bbox> ball_position;
    std::vector<darknet::RelateiveBBox> ball_position_relative;
    darknet::obj_detection(net_, &raw_img, parameters.ball.low_thresh, ball_position_relative);
    if (CvtRelativePosition(ball_position_relative, ball_position)) {
        float max_prob = 0.0;
        for (auto bbox : ball_position) {
            ROS_DEBUG("find %5d - %5f - (%d, %d) && (%d, %d)", bbox.m_label, bbox.m_prob, bbox.m_left, bbox.m_top, bbox.m_right, bbox.m_bottom);
            if (bbox.m_label == 0 && bbox.m_prob > max_prob) {
                m_data.see_ball = true;
                m_data.ball_image.x = (bbox.m_left + bbox.m_right) / 2.0;
                m_data.ball_image.y = (bbox.m_top + bbox.m_bottom) / 2.0;
                max_prob = bbox.m_prob;
            }
        }
        cv::Point2f ball_field;
        if (m_projection.getOnRealCoordinate(cv::Point(m_data.ball_image.x, m_data.ball_image.y), ball_field)) {
            m_data.ball_field.x = ball_field.x;
            m_data.ball_field.y = ball_field.y;
        }
    }
    return m_data.see_ball;
}

bool
BallDetector::CvtRelativePosition(std::vector<darknet::RelateiveBBox>& ball_position, std::vector<darknet::bbox>& ball_position_cvt)
{
    for (auto rbbox : ball_position) {
        int left = (rbbox.m_x - rbbox.m_w / 2.) * parameters.camera.width;
        int right = (rbbox.m_x + rbbox.m_w / 2.) * parameters.camera.width;
        int top = (rbbox.m_y - rbbox.m_h / 2.) * parameters.camera.height;
        int bottom = (rbbox.m_y + rbbox.m_h / 2.) * parameters.camera.height;

        if (left < 0)
            left = 0;
        if (right > parameters.camera.width - 1)
            right = parameters.camera.width - 1;
        if (top < 0)
            top = 0;
        if (bottom > parameters.camera.height - 1)
            bottom = parameters.camera.height - 1;
        ball_position_cvt.emplace_back(rbbox.m_label, rbbox.m_prob, left, top, right, bottom);
    }
    return true;
}

} // namespace dvision
