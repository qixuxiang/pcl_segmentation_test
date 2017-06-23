// Created on: May 20, 2017
//     Author: Wenxing Mei <mwx36mwx@gmail.com>

#include "dvision/dvision.hpp"
#include "dconfig/dconstant.hpp"

namespace dvision {
static const int VISION_FREQ = 30;
DVision::DVision(ros::NodeHandle* n)
  : DProcess(VISION_FREQ, false)
  , m_nh(n)
{
    parameters.init(n);
    m_projection.init(n);

    m_ball.Init();
    m_circle.Init();
    m_field.Init();
    m_goal.Init();
    m_line.Init();
    m_loc.Init();

    ROS_INFO("%lf %lf", parameters.camera.centerInUndistX, parameters.camera.centerInUndistY);
    m_ball_tracker.Init(parameters.camera.extrinsic_para,
                        parameters.camera.fx,
                        parameters.camera.fy,
                        parameters.camera.undistCx,
                        parameters.camera.undistCy,
                        parameters.camera.centerInUndistX,
                        parameters.camera.centerInUndistY
    );

    // FIXME(MWX): switch
    Frame::initEncoder();

    // FIXME(MWX): parallel!
    m_concurrent.push([] {
        //     ROS_INFO("concurrent");
    });
    m_sub_motion_info = m_nh->subscribe("/humanoid/MotionInfo", 1, &DVision::motionCallback, this);
    m_sub_behaviour_info = m_nh->subscribe("/humanoid/BehaviorInfo", 1, &DVision::behaviourCallback, this);
    m_sub_reload_config = m_nh->subscribe("/humanoid/ReloadVisionConfig", 1, &DVision::reloadConfigCallback, this);
    m_pub = m_nh->advertise<VisionInfo>("/humanoid/VisionInfo", 1);
    m_deltaClient = m_nh->serviceClient<dmotion::GetDelta>("getDelta");

    if(parameters.simulation) {
        m_transmitter = new dtransmit::DTransmit("127.0.0.1");
    } else {
        m_transmitter = new dtransmit::DTransmit(parameters.udpBroadcastAddress);
    }
    if (parameters.simulation) {
        ROS_INFO("Simulation mode!");
        m_transmitter->addRosRecv<VisionInfo>(dconstant::network::monitorBroadcastAddressBase + parameters.robotId, [&](VisionInfo& msg) {
            m_data = msg;
            //ROS_INFO_STREAM(m_data.robot_pos);
        });
    } else {
        CameraSettings s(n);
        m_camera = new Camera(s);
        ROS_INFO("Not simulation mode!");
    }

    m_transmitter->startService();
}

DVision::~DVision()
{
}

void
DVision::tick()
{
    auto begin = ros::Time::now();
    ROS_DEBUG("dvision tick");

    /**********
     * Update *
     **********/

    // update delta
//    dmotion::GetDelta srv;
//    if(m_deltaClient.call(srv)) {
//        auto d = srv.response.delta;
//        // TODO(yuthon): Here we got delta data.
//    }

    m_projection.updateExtrinsic(m_pitch, m_yaw);

    if (!parameters.simulation) {
        auto frame = m_camera->capture();
        m_data = VisionInfo();

        if (!m_loc.Update(m_projection)) {
            // ROS_ERROR("Cannot update localization!");
        }

        // get image in BGR and HSV color space
        m_gui_img = frame.getBGR_raw();
        m_hsv_img = frame.getHSV();

        /******************
         * Field Detector *
         ******************/

        m_data.see_field = m_field.Process(m_hsv_img, m_gui_img, m_projection);

        if (parameters.field.enable && !m_data.see_field) {
            // ROS_ERROR("Detecting field failed.");
        }

        /*****************
         * Line Detector *
         *****************/

        m_data.see_line = m_line.Process(m_canny_img, m_hsv_img, m_gui_img, m_field.field_convex_hull(), m_field.field_binary_raw(), m_projection);

        if (!m_data.see_line) {
            // ROS_ERROR("Detecting lines failed.");
        }

        /*******************
         * Circle detector *
         *******************/

        if (m_data.see_field && m_data.see_line) {
            m_data.see_circle = m_circle.Process(m_line.clustered_lines());
        }

        //        if (m_data.see_circle) {
        //            if (m_ball_tracker.Process(m_circle.result_circle().x, m_circle.result_circle().y, static_cast<double>(m_pitch), static_cast<double>(m_yaw))) {
        //                m_data.cmd_head_ball_track.x = 0;
        //                m_data.cmd_head_ball_track.y = m_ball_tracker.m_out_pitch / M_PI * 180;
        //                m_data.cmd_head_ball_track.z = m_ball_tracker.m_out_yaw / M_PI * 180;
        //                // cout << "c_pitch: " << m_data.cmd_head_ball_track.y << endl;
        //                // cout << "c_yaw: " << m_data.cmd_head_ball_track.z << endl;
        //            }
        //        }
        /*****************
         * Goal Detector *
         *****************/

        if (m_data.see_field) {
            m_data.see_goal = m_goal.Process(m_canny_img, m_hsv_img, m_gui_img, m_field.hull_field(), m_projection);
        }

        /****************
         * Localization *
         ****************/
        m_loc_img = cv::Mat(500, 500, CV_8UC3, cv::Scalar(0, 0, 0));

        if (m_data.see_field && m_data.see_line) {
            m_data.loc_ok = m_loc.Calculate(m_line.clustered_lines(),
                                            m_data.see_circle,
                                            m_field.field_hull_real_center(),
                                            m_field.field_hull_real(),
                                            m_field.field_hull_real_rotated(),
                                            m_circle.result_circle(),
                                            m_goal.goal_position(),
                                            m_loc_img,
                                            m_projection);
        }

        /*****************
         * Ball Detector *
         *****************/
        ros::Time begin = ros::Time::now();

        m_data.see_ball = m_ball.GetBall(frame.getBGR_raw(), m_gui_img, m_projection);
        ros::Duration end = ros::Time::now() - begin;
        std::cout << "------------Ball: " << end.toNSec() / 1000000.0 << "ms-----------" << std::endl;

        /*******************
         * Post-Processing *
         *******************/

        prepareVisionInfo(m_data);
        showDebugImg();
    }
    updateViewRange();
    trackBall();

    m_pub.publish(m_data);
    m_transmitter->sendRos(dconstant::network::robotBroadcastAddressBase + parameters.robotId, m_data);

    m_concurrent.spinOnce();
    m_concurrent.join();
    auto end = ros::Time::now();
    auto d = (end - begin).toSec();
    ROS_DEBUG("dvision tick used %lf", d);
}

void
DVision::motionCallback(const dmotion::MotionInfo::ConstPtr& motion_msg)
{
    m_motion_info = *motion_msg;

    m_loc.CalcDeadReckoning(m_motion_info);
    m_pitch = m_motion_info.action.headCmd.pitch;
    m_yaw = m_motion_info.action.headCmd.yaw;
//    ROS_INFO_STREAM(m_motion_info.action.headCmd);
}

void
DVision::behaviourCallback(const dbehavior::BehaviourInfo::ConstPtr& behaviour_msg)
{
    m_behaviour_info = *behaviour_msg;
    if (m_behaviour_info.save_image) {
        auto frame = m_camera->capture();
        std::string path_str;
        path_str = "p_" + std::to_string(m_pitch) + "_y_" + std::to_string(m_yaw) + " ";
        ROS_INFO("save_image! %s", path_str.c_str());
        frame.save(path_str);
    }
}

void
DVision::reloadConfigCallback(const std_msgs::String::ConstPtr&)
{
    parameters.update();
}

void
DVision::prepareVisionInfo(VisionInfo& m_data)
{
    // localization
    m_data.robot_pos.x = m_loc.location().x;
    m_data.robot_pos.y = m_loc.location().y;
    m_data.robot_pos.z = m_loc.location().z;
    // goal
    if (m_data.see_goal) {
        std::vector<cv::Point2f> goal_global = getOnGlobalCoordinate(m_loc.location(), m_goal.goal_position());
        if (goal_global.size() >= 1) {
            m_data.left_goal.x = goal_global[0].x;
            m_data.left_goal.y = goal_global[0].y;
            if (goal_global.size() == 2) {
                m_data.see_both_goal = true;
                m_data.right_goal.x = goal_global[1].x;
                m_data.right_goal.y = goal_global[1].y;
            } else if (goal_global.size() == 3) {
                m_data.see_unknown_goal = true;
                m_data.unknown_goal.x = goal_global[2].x;
                m_data.unknown_goal.y = goal_global[2].y;
            }
        }
    }

    if (m_data.see_circle) {
        cv::Point2d circle_global = getOnGlobalCoordinate(m_loc.location(), m_circle.result_circle());
        m_data.circle_field.x = circle_global.x;
        m_data.circle_field.y = circle_global.y;
    }

    // lines
    if (m_data.see_line) {
        std::vector<LineSegment> lines_global = getOnGlobalCoordinate(m_loc.location(), m_line.clustered_lines());
        m_data.lines.resize(lines_global.size());
        for (uint32_t i = 0; i < lines_global.size(); ++i) {
            auto& p1 = lines_global[i].P1;
            m_data.lines[i].endpoint1.x = p1.x;
            m_data.lines[i].endpoint1.y = p1.y;

            auto& p2 = lines_global[i].P2;
            m_data.lines[i].endpoint2.x = p2.x;
            m_data.lines[i].endpoint2.y = p2.y;
        }
    }

    // ball
    if (m_data.see_ball) {
        ROS_INFO("ball field: %lf %lf %lf", m_data.ball_field.x, m_data.ball_field.y, m_data.ball_field.z);

        m_data.ball_image.x = m_ball.ball_image().x;
        m_data.ball_image.y = m_ball.ball_image().y;
        m_data.ball_field.x = m_ball.ball_field().x;
        m_data.ball_field.y = m_ball.ball_field().y;
        cv::Point2f ball_global = getOnGlobalCoordinate(m_data.robot_pos, cv::Point2f(m_data.ball_field.x, m_data.ball_field.y));
        m_data.ball_global.x = ball_global.x;
        m_data.ball_global.y = ball_global.y;
    }
}

void
DVision::updateViewRange()
{
    // TODO(MWX): switch on/off
    // viewRange, four
    cv::Point2f upperLeft;
    cv::Point2f upperRight;
    cv::Point2f lowerLeft;
    cv::Point2f lowerRight;

    m_projection.getOnRealCoordinate(cv::Point(0, 0), upperLeft);
    m_projection.getOnRealCoordinate(cv::Point(parameters.camera.width - 1, 0), upperRight);
    m_projection.getOnRealCoordinate(cv::Point(0, parameters.camera.height - 1), lowerLeft);
    m_projection.getOnRealCoordinate(cv::Point(parameters.camera.width - 1, parameters.camera.height - 1), lowerRight);

//    LineSegment l1(upperLeft, lowerLeft);
//    LineSegment l2(upperRight, lowerRight);
//    cv::Point2d cross;
//    bool intersect = l1.Intersect(l2, cross);
//    if(intersect) {
//        upperLeft *= -100;
//        upperRight *= -100;
//    }

    auto v1 = upperLeft - lowerLeft;
    auto v2 = lowerRight - lowerLeft;
    auto theta = getAngleBetweenVectors(v1, v2);
    if(theta < 180 && theta > 0) {
        upperLeft *= -100;
    }

    v1 = upperRight - lowerRight;
    v2 = - v2;
    theta = getAngleBetweenVectors(v2, v1);
    if(theta < 180 && theta > 0) {
        upperRight *= -100;
    }

    upperLeft = getOnGlobalCoordinate(m_data.robot_pos, upperLeft);
    upperRight = getOnGlobalCoordinate(m_data.robot_pos, upperRight);
    lowerLeft = getOnGlobalCoordinate(m_data.robot_pos, lowerLeft);
    lowerRight = getOnGlobalCoordinate(m_data.robot_pos, lowerRight);

    m_data.viewRange.resize(4);
    m_data.viewRange[0].x = upperLeft.x;
    m_data.viewRange[0].y = upperLeft.y;

    m_data.viewRange[1].x = upperRight.x;
    m_data.viewRange[1].y = upperRight.y;

    m_data.viewRange[2].x = lowerRight.x;
    m_data.viewRange[2].y = lowerRight.y;

    m_data.viewRange[3].x = lowerLeft.x;
    m_data.viewRange[3].y = lowerLeft.y;
}

void
DVision::showDebugImg()
{
    if (parameters.monitor.update_loc_img) {
        cv::namedWindow("loc", CV_WINDOW_NORMAL);
        cv::imshow("loc", m_loc_img);
    }

    if (parameters.monitor.update_canny_img) {
        cv::namedWindow("canny", CV_WINDOW_NORMAL);
        cv::imshow("canny", m_canny_img);
    }

    if (parameters.monitor.update_field_binary) {
        m_field_binary = cv::Mat::zeros(m_hsv_img.size(), CV_8UC1);
        cv::inRange(m_hsv_img, cv::Scalar(parameters.field.h0, parameters.field.s0, parameters.field.v0), cv::Scalar(parameters.field.h1, parameters.field.s1, parameters.field.v1), m_field_binary);
        cv::namedWindow("field_binary", CV_WINDOW_NORMAL);
        cv::imshow("field_binary", m_field_binary);
    }

    if (parameters.monitor.update_goal_binary) {
        m_goal_binary = cv::Mat::zeros(m_hsv_img.size(), CV_8UC1);
        cv::inRange(m_hsv_img, cv::Scalar(parameters.goal.h0, parameters.goal.s0, parameters.goal.v0), cv::Scalar(parameters.goal.h1, parameters.goal.s1, parameters.goal.v1), m_goal_binary);
        cv::namedWindow("goal_binary", CV_WINDOW_NORMAL);
        cv::imshow("goal_binary", m_goal_binary);
    }

    if (parameters.monitor.update_line_binary) {
        m_line_binary = cv::Mat::zeros(m_hsv_img.size(), CV_8UC1);
        cv::inRange(m_hsv_img, cv::Scalar(parameters.line.h0, parameters.line.s0, parameters.line.v0), cv::Scalar(parameters.line.h1, parameters.line.s1, parameters.line.v1), m_line_binary);
        cv::namedWindow("line_binary", CV_WINDOW_NORMAL);
        cv::imshow("line_binary", m_line_binary);
    }

    if (parameters.monitor.update_gui_img) {
//        cv::namedWindow("gui", CV_WINDOW_NORMAL);
//        cv::imshow("gui", m_gui_img);
//        cv::waitKey(1);
         int len;
         auto buf = Frame::encode(m_gui_img, len);
         m_transmitter->sendRaw(dconstant::network::robotGuiBase + parameters.robotId, buf.get(), len);
    }
}

void DVision::trackBall() {
    // ball
    if (m_data.see_ball) {
        // track ball
        if (m_ball_tracker.Process(m_data.ball_field.x, m_data.ball_field.y, Degree2Radian(m_pitch), Degree2Radian(m_yaw))) {
            m_data.ballTrack.pitch = Radian2Degree(m_ball_tracker.out_pitch());
            m_data.ballTrack.yaw = Radian2Degree(m_ball_tracker.out_yaw());
        }
    }
}
} // namespace dvision
