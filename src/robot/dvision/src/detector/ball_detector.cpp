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
    // // parse label list
    // if (!std::ifstream(parameters.ball.label_file)) {
    //     ROS_ERROR("Darknet label file doesn't exist!");
    //     return false;
    // }
    // label_list = darknet::get_labels(parameters.ball.label_file);
    //
    // // parse net cfg and setup network
    // if (!std::ifstream(parameters.ball.net_cfg)) {
    //     ROS_ERROR("Darknet network cfg doesn't exist!");
    //     return false;
    // }
    //
    // // setup network
    // net_ = &(darknet::parse_network_cfg(parameters.ball.net_cfg));
    // darknet::params p = net_->get_params();
    // ROS_DEBUG("Network Setup: num_layers = %d, batch = %d\n", m_net->num_layers(), p.batch);
    //
    // // load pretrained weights
    // if (!std::ifstream(parameters.ball.weight_file)) {
    //     ROS_ERROR("Darknet weigth file doesn't exist!");
    //     return false;
    // }
    // darknet::load_weights(net_, parameters.ball.weight_file);
    //
    // // set batch to 1 for network inference
    // net_->set_network_batch(1);

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
