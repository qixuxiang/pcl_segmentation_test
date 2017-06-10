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
    // delete net_;
}

bool
BallDetector::GetBall()
{
    return true;
}

} // namespace dvision
