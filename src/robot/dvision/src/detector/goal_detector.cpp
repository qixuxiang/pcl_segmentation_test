/**
 * @Author: Yusu Pan <corenel>
 * @Date:   2017-06-02T19:47:02+08:00
 * @Email:  xxdsox@gmail.com
 * @Project: humanoid
 * @Filename: goal_detector.cpp
 * @Last modified by:   corenel
 * @Last modified time: 2017-06-02T19:48:11+08:00
 * @Copyright: ZJUDancer
 */

#include "dvision/goal_detector.hpp"
#include "dvision/parameters.hpp"

namespace dvision {
GoalDetector::GoalDetector()
{
}

bool
GoalDetector::Init()
{
    ROS_INFO("GoalDetector Init() started");
    return true;
}

GoalDetector::~GoalDetector()
{
}

bool
GoalDetector::GetPosts(cv::Mat& canny_img,
                       cv::Mat& raw_hsv,
                       cv::Mat& gray,
                       const cv::Mat& binary_frame,
                       Projection& projection,
                       const std::vector<cv::Point>& field_hull,
                       std::vector<LineSegment>& res_lines,
                       std::vector<LineSegment>& all_lines,
                       std::vector<cv::Point2f>& goal_position,
                       const bool& SHOWGUI,
                       cv::Mat& gui_img)
{
    // never use
    // if (binary_frame.empty()) {
    //   return false;
    // }
    cv::Rect rec;
    rec.x = 0;
    rec.y = 0;
    rec.width = parameters.camera.width;
    rec.height = parameters.camera.height;
    const int MinLineLength = parameters.goal.MinLineLength;
    const int DistanceToMerge = parameters.goal.DistanceToMerge;
    const int MaxOutField = parameters.goal.MaxOutField;
    const int MinNearFieldUpPoint = -20;

    std::vector<cv::Vec4i> linesP;

    // void HoughLinesP(InputArray image, OutputArray lines, double rho, double
    // theta, int threshold, double minLineLength=0, double maxLineGap=0 )
    HoughLinesP(canny_img, linesP, 1, M_PI / 45, 10, MinLineLength, 10);
    std::vector<LineSegment> all_ver_lines;
    for (size_t i = 0; i < linesP.size(); i++) {
        // lP (x_1, y_1, x_2, y_2)
        cv::Vec4i lP = linesP[i];
        LineSegment tmp_line(cv::Point2d(lP[0], lP[1]), cv::Point2d(lP[2], lP[3]));
        // std::vector<cv::Point> goalPostPoints;
        // 与竖直线夹角小于15度
        if (tmp_line.GetAbsMinAngleDegree(LineSegment(cv::Point(0, 0), cv::Point(0, 100))) < 15) {
            // 画出所有Hough到的竖直线
            // cv::line(gui_img, tmp_line.P1, tmp_line.P2, darkOrangeColor(), 1);

            double left_avg = 0;
            double right_avg = 0;

            // 取2^5+1=33个等分点
            std::vector<cv::Point2d> midds = tmp_line.GetMidPoints(5);
            int max_mid_point = midds.size();
            int vote_for_double_left = 0;
            int vote_for_double_right = 0;

            // 取下端点
            cv::Point down = (tmp_line.P1.y < tmp_line.P2.y) ? tmp_line.P2 : tmp_line.P1;
            double jump_double = parameters.goal.jumpMax;

            if (!CheckDownPointDistance(down, jump_double, projection, field_hull)) {
                continue;
            }

            for (size_t j = 0; j < midds.size(); j++) {
                if (!VoteGoalPostPoint(tmp_line, midds[j], jump_double, SHOWGUI, gui_img, raw_hsv, canny_img, left_avg, right_avg, vote_for_double_left, vote_for_double_right)) {
                    continue;
                }
            }

            bool left_OK = (vote_for_double_left / static_cast<float>(max_mid_point)) * 100. > parameters.goal.doubleVote;

            bool right_OK = (vote_for_double_right / static_cast<float>(max_mid_point)) * 100. > parameters.goal.doubleVote;

            if (left_OK || right_OK) {
                {
                    LineSegment tmp_line_changed = tmp_line;

                    // 球门柱下端点下探避免
                    // {
                    //   cv::Point validDown =
                    //       (tmp_line.P1.y < tmp_line.P2.y) ? tmp_line.P2 : tmp_line.P1;
                    //
                    //   // 按y升序排列
                    //   sort(goalPostPoints.begin(), goalPostPoints.end(),
                    //        [](const cv::Point &lhs, const cv::Point &rhs) {
                    //          return lhs.y < rhs.y;
                    //        });
                    //   // 如果line的下端点与高响应点距离大于10
                    //   // 则将line的下端点换为高响应点集合的下端点
                    //   if (GetDistance(goalPostPoints.back(), validDown) > 10) {
                    //     validDown = goalPostPoints.back();
                    //   }
                    //   // 如果一个高响应点与其下的高响应点的距离大于10
                    //   // 即出现goal post下探到禁区线的情况
                    //   // 则将line的下端点换为该高响应点
                    //   for (size_t l = 0; l < goalPostPoints.size() - 1; ++l) {
                    //     if (GetDistance(goalPostPoints[l], goalPostPoints[l + 1]) >=
                    //     10) {
                    //       // cout << "orig down (" << validDown.x << "," <<
                    //       validDown.y
                    //       //      << ") / valid down (" << goalPostPoints[l].x << ","
                    //       //      << goalPostPoints[l].y << ")" << endl;
                    //       validDown = goalPostPoints[l];
                    //     }
                    //   }
                    //   // 替换line的下端点
                    //   if (tmp_line_changed.P1.y < tmp_line_changed.P2.y) {
                    //     tmp_line_changed.P2 = validDown;
                    //   } else {
                    //     tmp_line_changed.P1 = validDown;
                    //   }
                    // }

                    // 平移，将球门柱左边缘线或右边缘线向中间平移
                    if (left_OK) {
                        int amount = abs(left_avg / vote_for_double_left) / 2.;
                        tmp_line_changed.P1.x -= amount;
                        tmp_line_changed.P2.x -= amount;
                    } else if (right_OK) {
                        int amount = abs(right_avg / vote_for_double_right) / 2.;
                        tmp_line_changed.P1.x += amount;
                        tmp_line_changed.P2.x += amount;
                    }
                    tmp_line_changed.Clip(rec);

                    all_ver_lines.push_back(tmp_line_changed);
                }
            }
        }
    }

    // 线段融合
    std::vector<LineSegment> all_ver_lines_2;
    MergeLinesMax(all_ver_lines, 30, DistanceToMerge, all_ver_lines_2, rec);

    for (size_t i = 0; i < all_ver_lines_2.size(); i++) {
        LineSegment tmp_line = all_ver_lines_2[i];
        cv::Point up = (tmp_line.P1.y > tmp_line.P2.y) ? tmp_line.P2 : tmp_line.P1;
        cv::Point down = (tmp_line.P1.y < tmp_line.P2.y) ? tmp_line.P2 : tmp_line.P1;

        double ver_len = tmp_line.GetLength();
        // 线片段上端点须在场地凸包外一定距离
        if (pointPolygonTest(field_hull, up, true) > MinNearFieldUpPoint) {
            // cout << "Goal: up in field" << endl;
            continue;
        }

        cv::Point2f down_real;
        if (!projection.getOnRealCoordinate(down, down_real)) {
            // ROS_ERROR("Erorr in programming!");
            return false;
        }

        // 根本不知道在干什么
        if (!CheckDistanceBox(down_real, ver_len)) {
            // cout << "Goal: check box error" << endl;
            continue;
        }
        // 线片段下端点须在场地凸包内
        double downDistance2Field = pointPolygonTest(field_hull, down, true);
        if (downDistance2Field < MaxOutField) {
            // cout << "Goal: down not in field" << endl;
            continue;
        }
        // 线片段与机器人距离大于5m亦舍弃
        if (GetDistance(down_real) > 750) {
            // cout << "Goal: too far from robot" << endl;
            continue;
        }

        // 对于符合上述条件的goal post，从下端点开始进行下探
        // 直到没有高响应的点
        int cnt_invalid_points = 0;
        int cnt_total_ext_points = 0;

        // invalid point出现五个或者总计延长点超过15个
        // 则终止延长
        while (cnt_invalid_points <= 5 && cnt_total_ext_points <= 15) {
            // 取tmpLine向下的延长线上的一点
            cv::Point2d down_extension_point = tmp_line.ExtensionPointDown(2);

            // 检查延长点与机器人距离，并获取jumpDouble
            double jump_double = parameters.goal.jumpMax;
            if (!CheckDownPointDistance(down_extension_point, jump_double, projection, field_hull)) {
                continue;
            }
            // 检查高响应
            double left_avg = 0;
            double right_avg = 0;
            int vote_for_double_left = 0;
            int vote_for_double_right = 0;
            if (!VoteGoalPostPoint(tmp_line, down_extension_point, jump_double, SHOWGUI, gui_img, raw_hsv, canny_img, left_avg, right_avg, vote_for_double_left, vote_for_double_right)) {
                continue;
            }
            bool left_OK = (vote_for_double_left / 1.) * 100. > parameters.goal.doubleVote / 2;
            bool right_OK = (vote_for_double_right / 1.) * 100. > parameters.goal.doubleVote / 2;

            // 若左右侧出现足够多的高响应点，则该延长点valid
            if (left_OK || right_OK) {
                tmp_line.SetDownPoint(down_extension_point);
                // cout << "Expand to (" << down_extension_point.x << ","
                //      << down_extension_point.y << ")" << endl;
            } else {
                // cout << "Invalid (" << down_extension_point.x << ","
                //      << down_extension_point.y << ")" << endl;
                cnt_invalid_points++;
            }
            cnt_total_ext_points++;
        }

        // 获取当前的下端点以及其对应的机器人坐标系中的点
        down = tmp_line.GetDownPoint();
        if (!projection.getOnRealCoordinate(down, down_real)) {
            // ROS_ERROR("Erorr in programming!");
            return false;
        }

        // 令人感动球门柱下端点加了进去
        goal_position.push_back(down_real);
        // 顺便加了整个球门柱的线片段
        res_lines.push_back(LineSegment(down, up));
    }

    if (SHOWGUI && parameters.goal.showVote) {
        for (size_t i = 0; i < all_ver_lines_2.size(); i++) {
            cv::line(gui_img, all_ver_lines_2[i].P1, all_ver_lines_2[i].P2, darkOrangeColor(), 1);
        }
    }
    if (SHOWGUI && parameters.goal.showResult) {
        for (size_t i = 0; i < res_lines.size(); i++) {
            cv::line(gui_img, res_lines[i].P1, res_lines[i].P2, blueColor(), 2);
        }
    }
    return res_lines.size() > 0;
}

bool
GoalDetector::CheckDistanceBox(cv::Point2f down_point_in_real, double length)
{
    LineSegment lower_bound(cv::Point2f(parameters.goal.NearestDistance, parameters.goal.NearMinLen), cv::Point2f(parameters.goal.FurthestDistance, parameters.goal.FarMinLen));
    LineSegment higher_bound(cv::Point2f(parameters.goal.NearestDistance, parameters.goal.NearMaxLen), cv::Point2f(parameters.goal.FurthestDistance, parameters.goal.FarMaxLen));
    LinearBoundaryChecker checker(lower_bound, higher_bound);

    double distance_to_robot = GetDistance(down_point_in_real);

    return checker.CheckInside(distance_to_robot, length);
}

bool
GoalDetector::CheckDownPointDistance(const cv::Point& down, double& jump_double, Projection& projection, std::vector<cv::Point> field_hull)
{
    // 该端点在机器人坐标系中的位置
    cv::Point2f down_real;
    if (!projection.getOnRealCoordinate(down, down_real)) {
        // ROS_ERROR("Erorr in programming!");
        return false;
    }
    double down_distance_to_field = pointPolygonTest(field_hull, down, true);
    if (down_distance_to_field < parameters.goal.MaxOutField)
        return false;
    // 根据down点与机器人的距离来选择不同的jump值
    double distance = GetDistance(down_real);
    if (distance < 200) {
        jump_double = 40 * 1920 / 640;
    } else if (distance >= 200 && distance < 300) {
        jump_double = 23 * 1920 / 640;
    } else {
        jump_double = 15 * 1920 / 640;
    }

    return true;
}

bool
GoalDetector::VoteGoalPostPoint(LineSegment& tmp_line,
                                const cv::Point2d& point,
                                const double& jump_double,
                                const bool& SHOWGUI,
                                cv::Mat& gui_img,
                                const cv::Mat& raw_hsv,
                                const cv::Mat& canny_img,
                                double& left_avg,
                                double& right_avg,
                                int& vote_for_double_left,
                                int& vote_for_double_right)
{
    // 该点时候是合格的goalPostPoint的flag
    // bool validGoalPostPoint = false;
    // 从某个等分点上取垂线段
    LineSegment to_check = tmp_line.PerpendicularLineSegment(jump_double, point);

    // 左右垂线段片段
    cv::Point left = (to_check.P1.x < to_check.P2.x) ? to_check.P1 : to_check.P2;
    cv::Point right = (to_check.P1.x < to_check.P2.x) ? to_check.P2 : to_check.P1;
    cv::LineIterator it_left(canny_img, point, left, 8);
    cv::LineIterator it_right(canny_img, point, right, 8);
    cv::LineIterator it_hsv_left(raw_hsv, point, left, 8);
    cv::LineIterator it_hsv_right(raw_hsv, point, right, 8);

    int safe_to_show = 0;
    if (to_check.P1.x >= 0 && to_check.P1.y >= 0 && to_check.P1.x < parameters.camera.width && to_check.P1.y < parameters.camera.height) {
        safe_to_show++;
    }
    if (to_check.P2.x >= 0 && to_check.P2.y >= 0 && to_check.P2.x < parameters.camera.width && to_check.P2.y < parameters.camera.height) {
        safe_to_show++;
    }

    for (int k = 0; k < it_left.count; k++, ++it_left, ++it_hsv_left) {
        if (k < 2)
            continue;
        uchar val = *(*it_left);
        cv::Vec3b hsvC = (cv::Vec3b)*it_hsv_left;

        // Canny高响应（如球门柱左边缘线上的某个等分点的垂线段碰到了右边缘线）
        // 且左右边缘线像素距离大于minDoubleLength
        if (val > 0 && k > parameters.goal.minDoubleLength) {
            // 在guiImg上画出来
            if (safe_to_show >= 2 && SHOWGUI && parameters.goal.showVote) {
                cv::line(gui_img, point, it_hsv_left.pos(), redColor(), 1);
            }
            // 用以之后计算平移距离
            left_avg += k;
            // 投票也有我一份
            vote_for_double_left++;
            // validGoalPostPoint = true;
            break;
        }

        // 像素点的HSV不在预设范围内则gg
        if (hsvC[0] >= parameters.goal.h0 && hsvC[0] <= parameters.goal.h1 && hsvC[1] >= parameters.goal.s0 && hsvC[1] <= parameters.goal.s1 && hsvC[2] >= parameters.goal.v0 &&
            hsvC[2] <= parameters.goal.v1) {
        } else {
            break;
        }
    }

    // 同上
    for (int k = 0; k < it_right.count; k++, ++it_right, ++it_hsv_right) {
        if (k < 2)
            continue;
        uchar val = *(*it_right);
        cv::Vec3b hsvC = (cv::Vec3b)*it_hsv_right;

        if (val > 0 && k > parameters.goal.minDoubleLength) {
            if (safe_to_show >= 2 && SHOWGUI && parameters.goal.showVote) {
                cv::line(gui_img, point, it_hsv_right.pos(), redColor(), 1);
            }
            right_avg += k;
            vote_for_double_right++;
            // validGoalPostPoint = true;
            break;
        }

        if (hsvC[0] >= parameters.goal.h0 && hsvC[0] <= parameters.goal.h1 && hsvC[1] >= parameters.goal.s0 && hsvC[1] <= parameters.goal.s1 && hsvC[2] >= parameters.goal.v0 &&
            hsvC[2] <= parameters.goal.v1) {
        } else {
            break;
        }
    }

    // 如果left或者right满足条件，则将此点加入goalPostPoints中待用
    // if (validGoalPostPoint) {
    //   goalPostPoints.push_back(midds[j]);
    // }

    return true;
}

} // namespace dvision
