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
  , raw_img_(448, 448, 3, false)
  , ball_image_(0, 0)
  , ball_image_top_(0, 0)
  , ball_image_bottom_(0, 0)
  , ball_field_(0.0, 0.0)
  , ball_field_kalman_(0.0, 0.0)
  , kalmanI_(ball_field_kalman_)
{
}

bool
BallDetector::Init()
{
    // parse label list
    std::string label_file_path = parameters.ball.home_folder + "/" + parameters.ball.label_file;
    if (!std::ifstream(label_file_path)) {
        ROS_ERROR("darknet label file doesn't exist in %s!", label_file_path.c_str());
        return false;
    }
    label_list_ = darknet::get_labels(label_file_path);

    // parse net cfg and setup network
    std::string net_cfg_path = parameters.ball.home_folder + "/" + parameters.ball.net_cfg;

    if (!std::ifstream(net_cfg_path)) {
        ROS_ERROR("darknet network cfg doesn't exist in %s!", net_cfg_path.c_str());
        return false;
    }

    // setup network
    net_ = &(darknet::parse_network_cfg(net_cfg_path));
    darknet::params p = net_->get_params();
    ROS_INFO("network setup: num_layers = %d, batch = %d\n", net_->num_layers(), p.batch);

    // load pretrained weights
    std::string weight_file_path = parameters.ball.home_folder + "/" + parameters.ball.weight_file;
    if (!std::ifstream(weight_file_path)) {
        ROS_ERROR("darknet weigth file doesn't exist in %s!", weight_file_path.c_str());
        return false;
    }
    darknet::load_weights(net_, weight_file_path);

    // set batch to 1 for network inference
    net_->set_network_batch(1);

    ROS_DEBUG("BallDetector Init");
    return true;
}

BallDetector::~BallDetector()
{
    delete net_;
}

bool
BallDetector::Update()
{
    if (parameters.ball.useKalman) {
        kalmanI_.GetPrediction();
        cv::Point2d kal_res = kalmanI_.Update(cv::Point2d(ball_field_.x, ball_field_.y));
        ball_field_kalman_.x = kal_res.x;
        ball_field_kalman_.y = kal_res.y;
    }
}

bool
BallDetector::GetBall(const cv::Mat& frame, cv::Mat& gui_img, Projection& projection)
{
    // ROS_DEBUG("BallDetector Tick");
    bool see_ball = false;
    darknet::params p = net_->get_params();
    cv::Mat frame_resized;
    cv::Size size(p.w, p.h);

    cv::resize(frame, frame_resized, size);
    raw_img_.from_mat(frame_resized);

    std::vector<darknet::bbox> ball_position;
    std::vector<darknet::RelateiveBBox> ball_position_relative;

    darknet::obj_detection(net_, &raw_img_, parameters.ball.low_thresh, ball_position_relative);

    if (CvtRelativePosition(ball_position_relative, ball_position)) {
        float max_prob = 0.0;
        for (auto bbox : ball_position) {
            ROS_DEBUG("find %5d - %5f - (%d, %d) && (%d, %d)", bbox.m_label, bbox.m_prob, bbox.m_left, bbox.m_top, bbox.m_right, bbox.m_bottom);
            bool label_ok = bbox.m_label == 0;
            bool prob_ok = bbox.m_prob > max_prob;
            bool size_ok = std::abs(bbox.m_left - bbox.m_right) < parameters.camera.width * 0.5 && std::abs(bbox.m_top - bbox.m_bottom) < parameters.camera.height * 0.5;
            if (label_ok && prob_ok && size_ok) {
                see_ball = true;
                ball_image_.x = (bbox.m_left + bbox.m_right) / 2.0;
                ball_image_.y = (bbox.m_top + bbox.m_bottom) / 2.0;
                ball_image_top_ = cv::Point(bbox.m_left, bbox.m_top);
                ball_image_bottom_ = cv::Point(bbox.m_right, bbox.m_bottom);
                max_prob = bbox.m_prob;
            }
        }
        // TODO(MWX): 7.5 is ball diamater/2 as constant
        projection.getOnRealCoordinate(ball_image_, ball_field_, 7.5);
    }

    if (parameters.monitor.update_gui_img && parameters.ball.showResult && see_ball) {
        cv::rectangle(gui_img, ball_image_top_, ball_image_bottom_, cv::Scalar(0, 255, 0), 2);
    }

    return see_ball;
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

cv::Point
BallDetector::ball_image()
{
    return ball_image_;
}

cv::Point2f
BallDetector::ball_field()
{
    if (parameters.ball.useKalman) {
        return ball_field_kalman_;
    } else {
        return ball_field_;
    }
}

} // namespace dvision
