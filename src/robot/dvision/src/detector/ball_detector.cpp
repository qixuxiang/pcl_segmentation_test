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
BallDetector::GetBall(cv::Mat& frame, std::vector<darknet::bbox>& ball_position)
{
    darknet::Image raw_img(frame);
    darknet::params p = net_->get_params();
    ROS_INFO("resize to %d x %d x %d", p.h, p.w, p.c);
    raw_img.resize_neo(p.h, p.w, p.c);

    darknet::obj_detection(net_, &raw_img, parameters.ball.low_thresh, ball_position);
    for (auto bbox : ball_position) {
        ROS_INFO("%5d - %5f - (%d, %d) && (%d, %d)", bbox.m_label, bbox.m_prob, bbox.m_left, bbox.m_top, bbox.m_right, bbox.m_bottom);
        // std::cout << "left:" << bbox.m_left << std::endl;
        // std::cout << "right:" << bbox.m_right << std::endl;
        // std::cout << "top:" << bbox.m_top << std::endl;
        // std::cout << "bottom:" << bbox.m_bottom << std::endl;
    }

    return true;
}

} // namespace dvision
