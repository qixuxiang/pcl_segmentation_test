/**
 * @Author: Yusu Pan <corenel>
 * @Date:   2017-06-02T19:46:41+08:00
 * @Email:  xxdsox@gmail.com
 * @Project: humanoid
 * @Filename: line_detector.cpp
 * @Last modified by:   corenel
 * @Last modified time: 2017-06-02T19:48:18+08:00
 * @Copyright: ZJUDancer
 */

#include "dvision/line_detector.hpp"
#include "dvision/parameters.hpp"

namespace dvision {
LineDetector::LineDetector()
{
}

bool
LineDetector::Init()
{
    ROS_INFO("LineDetector Init() started");
    return true;
}

LineDetector::~LineDetector()
{
}

bool
LineDetector::GetLines(cv::Mat& raw_hsv, cv::Mat& field_mask, cv::Mat& gui_img, const bool& SHOWGUI, const cv::Mat& line_binary, std::vector<LineSegment>& res_lines)
{
    // true
    bool AprxDist = parameters.line.aprxDist;
    const int NUM_MID_P = 3;
    const int COUNT_MID_P = static_cast<int>(pow(2, NUM_MID_P) + 1);

    std::vector<cv::Vec4i> lines_from_houghP;
    // 概率霍夫线变换
    HoughLinesP(
      line_binary, lines_from_houghP, parameters.line.rhoHough, M_PI / parameters.line.thetaHough, parameters.line.threasholdHough, parameters.line.MinLineLength, parameters.line.maxLineGapHough);

    // TODO(corenel) show hough line in gui_img
    if (parameters.line.showAllLine && SHOWGUI) {
        for (size_t i = 0; i < lines_from_houghP.size(); i++) {
            cv::line(gui_img, cv::Point(lines_from_houghP[i][0], lines_from_houghP[i][1]), cv::Point(lines_from_houghP[i][2], lines_from_houghP[i][3]), redColor(), 1, 8);
        }
    }

    if (lines_from_houghP.size() < 1) {
        return false;
    }
    res_lines.reserve(lines_from_houghP.size());
    const int MIN_PIXEL = 2;
    // double LINE_WIDTH_CHECK = parameters.line.widthCheck;

    int jump_max = parameters.line.jumpMax;
    int jump_min = parameters.line.jumpMin;

    std::vector<cv::Point> query_point(lines_from_houghP.size() * COUNT_MID_P * 3);
    std::vector<cv::Point2f> real_query_point(lines_from_houghP.size() * COUNT_MID_P * 3);

    std::vector<int> both_end_size(lines_from_houghP.size() * COUNT_MID_P * 2);

    std::vector<LineSegment> perpendicular_LS(lines_from_houghP.size() * COUNT_MID_P * 3);

    std::vector<std::vector<cv::Point2d>> midds(lines_from_houghP.size(), std::vector<cv::Point2d>(COUNT_MID_P));
    std::vector<LineSegment> all_lines(lines_from_houghP.size());

    for (size_t i = 0; i < lines_from_houghP.size(); i++) {
        cv::Vec4i lP = lines_from_houghP[i];
        // 取出第一个线片段
        all_lines[i] = LineSegment(cv::Point2d(lP[0], lP[1]), cv::Point2d(lP[2], lP[3]));
        // 取出8等分点，就有9个端点
        midds[i] = all_lines[i].GetMidPoints(NUM_MID_P, false);
        for (size_t j = 0; j < COUNT_MID_P; j++) {
            int idx = static_cast<int>(i * COUNT_MID_P + j);
            // 垂直于原来线片段的小线片段 9 段, 向上20 和向下20
            perpendicular_LS[idx] = all_lines[i].PerpendicularLineSegment(jump_max, midds[i][j]);
            // 从等分点向上的线段迭代器
            cv::LineIterator it_up(line_binary, midds[i][j], perpendicular_LS[idx].P1, 8);
            // 从等分点向下的线段迭代器
            cv::LineIterator it_down(line_binary, midds[i][j], perpendicular_LS[idx].P2, 8);
            // 把等分点存在queryPoint[idx * 3]中
            query_point[idx * 3] = midds[i][j];
            if (AprxDist) {
                // 向上迭代找到一个最远距离为2px的点 存在queryPoint[idx * 3 + 1]中
                for (int k = 0; k < it_up.count; k++, ++it_up) {
                    cv::Point p = it_up.pos();
                    if (k <= MIN_PIXEL) {
                        query_point[idx * 3 + 1] = p;
                    } else {
                        break;
                    }
                }
                // 向下迭代找到一个最远实际距离为2px的点 存在queryPoint[idx * 3 + 2]中
                for (int k = 0; k < it_down.count; k++, ++it_down) {
                    cv::Point p = it_down.pos();
                    if (k <= MIN_PIXEL) {
                        query_point[idx * 3 + 2] = p;
                    } else {
                        break;
                    }
                }
            } else {
                // cv::Point2f centerOnReal;
                // if (!projection.GetOnRealCoordinate_single(query_point[idx * 3], centerOnReal)) {
                //     ROS_ERROR("Programming Error");
                //     return false;
                // }
                // // 向上迭代找到一个最远实际距离为LINE_WIDTH_CHECK的点
                // // 存在bothEndSize[idx * 2] 中 for (int k = 0; k < it_up.count; k++, ++it_up)
                // {
                //     cv::Point p = it_up.pos();
                //     both_end_size[idx * 2] = k;
                //     cv::Point2f pR;
                //     if (!projection.GetOnRealCoordinate_single(p, pR)) {
                //         ROS_ERROR("Programming Error");
                //         return false;
                //     }
                //     if (GetDistance(centerOnReal, pR) > LINE_WIDTH_CHECK) {
                //         break;
                //     }
                // }
                // // 向上迭代找到一个最远距离为LINE_WIDTH_CHECK的点
                // 存在bothEndSize[idx * 2 + 1] 中 for (int k = 0; k < it_down.count; k++, ++it_down)
                // {
                //     cv::Point p = it_down.pos();
                //     both_end_size[idx * 2 + 1] = k;
                //
                //     cv::Point2f pR;
                //     if (!projection.GetOnRealCoordinate_single(p, pR)) {
                //         ROS_ERROR("Programming Error");
                //         return false;
                //     }
                //     if (GetDistance(centerOnReal, pR) > LINE_WIDTH_CHECK) {
                //         break;
                //     }
                // }
            }
        }
    }

    // 获得实际坐标系中的点的位置
    if (AprxDist) {
        //    if (!projection.GetOnRealCoordinate(query_point, real_query_point))
        //    {
        //      ROS_ERROR("Programming Error");
        //      return false;
        //    }
    }

    uchar* data_img = raw_hsv.data;
    uchar* data_field_img = field_mask.data;
    for (size_t i = 0; i < all_lines.size(); i++) {
        double green_voter = 0;
        int vote_for_double_up = 0;
        int vote_for_double_down = 0;
        int vote_for_color_up = 0;
        int vote_for_color_down = 0;
        int not_valid_point = 0;

        // 遍历每个等分点
        for (size_t j = 0; j < COUNT_MID_P; j++) {
            int idx = i * COUNT_MID_P + j;
            int cur_up_array_size = 0;
            int cur_down_array_size = 0;
            cv::LineIterator it_up(line_binary, midds[i][j], perpendicular_LS[idx].P1, 8);
            cv::LineIterator it_down(line_binary, midds[i][j], perpendicular_LS[idx].P2, 8);
            cv::Point cenP = query_point[idx * 3];
            double up_distance = 0.0;
            double down_distance = 0.0;
            if (AprxDist) {
                cv::Point upP = query_point[idx * 3 + 1];
                cv::Point downP = query_point[idx * 3 + 2];

                // 获得向上向下点的距离
                up_distance = GetDistance(cenP, upP);
                down_distance = GetDistance(cenP, downP);

                // 获得向上向下实际点的距离
                // cv::Point2f cenPR = real_query_point[idx * 3];
                // cv::Point2f upPR = real_query_point[idx * 3 + 1];
                // cv::Point2f downPR = real_query_point[idx * 3 + 2];
                // double upDistanceR = GetDistance(cenPR, upPR);
                // double downDistanceR = GetDistance(cenPR, downPR);
                // 实际坐标系中的两个点的距离 × 图片中这两个点的距离 =
                // 要求距离line_width_check下的像素距离 ？ 难道实际坐标的单位是米？
                cur_up_array_size = it_up.count;
                cur_down_array_size = it_down.count;
                // TODO(yyj) implement IPM
                // cur_up_array_size = min(it_up.count, (int)((LINE_WIDTH_CHECK / upDistanceR) * up_distance));
                // cur_down_array_size = min(it_down.count, (int)((LINE_WIDTH_CHECK / downDistanceR) * down_distance));
            } else {
                cur_up_array_size = both_end_size[idx * 2];
                cur_down_array_size = both_end_size[idx * 2 + 1];
                up_distance = cur_up_array_size;
                down_distance = cur_down_array_size;
            }

            cur_up_array_size = std::max(jump_min, cur_up_array_size);
            cur_down_array_size = std::max(jump_min, cur_down_array_size);

            if (up_distance > 0.1 && down_distance > 0.1 && it_up.count >= jump_min && it_down.count >= jump_min) {
                cv::Point end_up, end_down;
                bool first_color = parameters.line.colorVUse;
                bool first_green = parameters.line.greenVUse;
                bool first_double = parameters.line.doubleVUse;

                int start_double = static_cast<int>(cur_up_array_size * parameters.line.doubleVStart);
                int end_double = static_cast<int>(cur_up_array_size * parameters.line.doubleVEnd);
                int start_green = static_cast<int>(cur_up_array_size * parameters.line.greenVStart);
                int end_green = static_cast<int>(cur_up_array_size * parameters.line.greenVEnd);
                int start_color = static_cast<int>(cur_up_array_size * parameters.line.colorVStart);
                int end_color = static_cast<int>(cur_up_array_size * parameters.line.colorVEnd);

                for (int k = 0; k < cur_up_array_size; k++, ++it_up) {
                    cv::Point p = it_up.pos();
                    end_up = p;
                    if (k > 0) {
                        // 行数×1920
                        int yPixel = p.y * parameters.camera.width;
                        // 指向该像素的指针
                        uchar* pixel = data_img + (yPixel + p.x) * 3;
                        uchar h = pixel[0];
                        uchar s = pixel[1];
                        uchar v = pixel[2];
                        uchar* pixelF = data_field_img + yPixel + p.x;
                        // 在每一小段垂直线段中，
                        // 一旦线段中的某个像素在lineBinary（canny算子结果中高响应）中是255，即是边缘线那么firstDouble
                        //= false;
                        if (first_double && k >= start_double && k <= end_double) {
                            if (*(*it_up) > 254) {
                                vote_for_double_up++;
                                first_double = false;
                            }
                        }
                        // 在每一小段垂直线段中，
                        // 一旦线段中的某个像素符合白线的hsv范围，就对vote_for_color_up加1，
                        // first_color = false;
                        if (first_color && k >= start_color && k <= end_color) {
                            if (h >= parameters.line.h0 && h <= parameters.line.h1 && s >= parameters.line.s0 && s <= parameters.line.s1 && v >= parameters.line.v0 && v <= parameters.line.v1) {
                                vote_for_color_up++;
                                first_color = false;
                            }
                        }
                        // 在每一小段垂直线段中，
                        // 一旦线段中的某个像素符合白线的hsv范围，就对greenVoter加0.5，
                        // first_color = false;
                        if (first_green && k >= start_green && k <= end_green) {
                            if (*pixelF > 254) {
                                green_voter += 0.5;
                                first_green = false;
                            }
                        }
                    }
                    //三个条件都符合了就不用继续做了
                    if (!parameters.line.showVote && !first_color && !first_green && !first_double) {
                        break;
                    }
                }
                first_color = parameters.line.colorVUse;
                first_green = parameters.line.greenVUse;
                first_double = parameters.line.doubleVUse;

                // 然后在itdowm中继续做这些事情
                for (int k = 0; k < cur_down_array_size; k++, ++it_down) {
                    cv::Point p = it_down.pos();
                    end_down = p;
                    if (k > 0) {
                        int ypixel = p.y * parameters.camera.width;
                        uchar* pixel = data_img + (ypixel + p.x) * 3;
                        uchar h = pixel[0];
                        uchar s = pixel[1];
                        uchar v = pixel[2];
                        uchar* pixelF = data_field_img + ypixel + p.x;
                        if (first_double && k >= start_double && k <= end_double) {
                            if (*(*it_down) > 254) {
                                vote_for_double_down++;
                                first_double = false;
                            }
                        }
                        if (first_color && k >= start_color && k <= end_color) {
                            if (h >= parameters.line.h0 && h <= parameters.line.h1 && s >= parameters.line.s0 && s <= parameters.line.s1 && v >= parameters.line.v0 && v <= parameters.line.v1) {
                                vote_for_color_down++;
                                first_color = false;
                            }
                        }
                        if (first_green && k >= start_green && k <= end_green) {
                            if (*pixelF > 254) {
                                green_voter += 0.5;
                                first_green = false;
                            }
                        }
                    }
                    if (!parameters.line.showVote && !first_color && !first_green && !first_double) {
                        break;
                    }
                }
                // 画出投票的结果，应该是好多小短线？
                if (SHOWGUI && parameters.line.showVote) {
                    cv::line(gui_img, end_up, end_down, yellowColor(), 1);
                    cv::circle(gui_img, end_up, 1, blueColor(), 1);
                    cv::circle(gui_img, end_down, 1, redColor(), 1);
                }
            } else {
                not_valid_point++;
            }
        }
        // 无效点数少于一半
        // bool checkIsValid = ((not_valid_point * 2) < COUNT_MID_P);
        bool checkIsValid = (COUNT_MID_P > (not_valid_point * 2));
        if (checkIsValid) {
            bool green_OK = true;
            bool up_double_OK = true;
            bool down_double_OK = true;
            bool up_color_OK = true;
            bool down_color_OK = true;
            double probability = 1;
            double total_VP = (COUNT_MID_P - not_valid_point);
            if (parameters.line.greenVUse) {
                int MIN_LINE_GREEN_VOTE = static_cast<int>((parameters.line.greenVote / 100.) * total_VP);
                green_OK = (green_voter >= MIN_LINE_GREEN_VOTE);
                probability *= (green_voter / total_VP);
            }
            if (parameters.line.doubleVUse) {
                int MIN_LINE_DOUBLE_VOTE = static_cast<int>((parameters.line.doubleVote / 100.) * total_VP);
                up_double_OK = (vote_for_double_up >= MIN_LINE_DOUBLE_VOTE);
                down_double_OK = (vote_for_double_down >= MIN_LINE_DOUBLE_VOTE);
                probability *= (std::max(vote_for_double_up, vote_for_double_down) / total_VP);
            }
            if (parameters.line.colorVUse) {
                int MIN_LINE_COLOR_VOTE = static_cast<int>((parameters.line.colorVote / 100.) * total_VP);
                up_color_OK = (vote_for_color_up >= MIN_LINE_COLOR_VOTE);
                down_color_OK = (vote_for_color_down >= MIN_LINE_COLOR_VOTE);
                probability *= (std::max(vote_for_color_up, vote_for_color_down) / total_VP);
            }
            // white line
            bool colorOK = (down_color_OK || up_color_OK);
            // canny
            bool doubleOK = (down_double_OK || up_double_OK);
            if (green_OK && colorOK && doubleOK) {
                all_lines[i].SetProbability(probability);
                res_lines.push_back(all_lines[i]);
            } else {
                if (parameters.line.showAllLine && SHOWGUI) {
                    cv::line(gui_img, all_lines[i].P1, all_lines[i].P2, redColor(), 1, 8);
                }
            }
        } else {
            if (parameters.line.showAllLine && SHOWGUI) {
                cv::line(gui_img, all_lines[i].P1, all_lines[i].P2, redMeloColor(), 1, 8);
            }
        }
    }
    return res_lines.size() > 0;
}
} // namespace dvision
