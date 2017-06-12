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
LineDetector::GetLines(cv::Mat& rawHSV, cv::Mat& fieldMask, cv::Mat& guiImg, const bool& showGui, const cv::Mat& lineBinary, std::vector<LineSegment>& resLines)
{
    // true
    bool aprxDist = parameters.line.aprxDist;
    const int NUM_MID_P = 3;
    const int COUNT_MID_P = static_cast<int>(pow(2, NUM_MID_P) + 1);

    std::vector<cv::Vec4i> linesFromHoughP;
    // 概率霍夫线变换
    HoughLinesP(
      lineBinary, linesFromHoughP, parameters.line.rhoHough, M_PI / parameters.line.thetaHough, parameters.line.threasholdHough, parameters.line.MinLineLength, parameters.line.maxLineGapHough);

    // TODO(corenel) show hough line in guiImg
    if (parameters.line.showAllLine && showGui) {
        for (size_t i = 0; i < linesFromHoughP.size(); i++) {
            cv::line(guiImg, cv::Point(linesFromHoughP[i][0], linesFromHoughP[i][1]), cv::Point(linesFromHoughP[i][2], linesFromHoughP[i][3]), redColor(), 1, 8);
        }
    }

    if (linesFromHoughP.size() < 1) {
        return false;
    }
    resLines.reserve(linesFromHoughP.size());
    const int MIN_PIXEL = 2;
    // double LINE_WIDTH_CHECK = parameters.line.widthCheck;

    int jumpMax = parameters.line.jumpMax;
    int jumpMin = parameters.line.jumpMin;

    std::vector<cv::Point> queryPoint(linesFromHoughP.size() * COUNT_MID_P * 3);
    std::vector<cv::Point2f> realQueryPoint(linesFromHoughP.size() * COUNT_MID_P * 3);

    std::vector<int> bothEndSize(linesFromHoughP.size() * COUNT_MID_P * 2);

    std::vector<LineSegment> PerpendicularLS(linesFromHoughP.size() * COUNT_MID_P * 3);

    std::vector<std::vector<cv::Point2d>> midds(linesFromHoughP.size(), std::vector<cv::Point2d>(COUNT_MID_P));
    std::vector<LineSegment> allLines(linesFromHoughP.size());

    for (size_t i = 0; i < linesFromHoughP.size(); i++) {
        cv::Vec4i lP = linesFromHoughP[i];
        // 取出第一个线片段
        allLines[i] = LineSegment(cv::Point2d(lP[0], lP[1]), cv::Point2d(lP[2], lP[3]));
        // 取出8等分点，就有9个端点
        midds[i] = allLines[i].GetMidPoints(NUM_MID_P, false);
        for (size_t j = 0; j < COUNT_MID_P; j++) {
            int idx = static_cast<int>(i * COUNT_MID_P + j);
            // 垂直于原来线片段的小线片段 9 段, 向上20 和向下20
            PerpendicularLS[idx] = allLines[i].PerpendicularLineSegment(jumpMax, midds[i][j]);
            // 从等分点向上的线段迭代器
            cv::LineIterator itUp(lineBinary, midds[i][j], PerpendicularLS[idx].P1, 8);
            // 从等分点向下的线段迭代器
            cv::LineIterator itDown(lineBinary, midds[i][j], PerpendicularLS[idx].P2, 8);
            // 把等分点存在queryPoint[idx * 3]中
            queryPoint[idx * 3] = midds[i][j];
            if (aprxDist) {
                // 向上迭代找到一个最远距离为2px的点 存在queryPoint[idx * 3 + 1]中
                for (int k = 0; k < itUp.count; k++, ++itUp) {
                    cv::Point p = itUp.pos();
                    if (k <= MIN_PIXEL) {
                        queryPoint[idx * 3 + 1] = p;
                    } else {
                        break;
                    }
                }
                // 向下迭代找到一个最远实际距离为2px的点 存在queryPoint[idx * 3 + 2]中
                for (int k = 0; k < itDown.count; k++, ++itDown) {
                    cv::Point p = itDown.pos();
                    if (k <= MIN_PIXEL) {
                        queryPoint[idx * 3 + 2] = p;
                    } else {
                        break;
                    }
                }
            } else {
                // cv::Point2f centerOnReal;
                // if (!projection.GetOnRealCoordinate_single(queryPoint[idx * 3], centerOnReal)) {
                //     ROS_ERROR("Programming Error");
                //     return false;
                // }
                // // 向上迭代找到一个最远实际距离为LINE_WIDTH_CHECK的点
                // // 存在bothEndSize[idx * 2] 中 for (int k = 0; k < itUp.count; k++, ++itUp)
                // {
                //     cv::Point p = itUp.pos();
                //     bothEndSize[idx * 2] = k;
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
                // 存在bothEndSize[idx * 2 + 1] 中 for (int k = 0; k < itDown.count; k++, ++itDown)
                // {
                //     cv::Point p = itDown.pos();
                //     bothEndSize[idx * 2 + 1] = k;
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
    if (aprxDist) {
        //    if (!projection.GetOnRealCoordinate(queryPoint, realQueryPoint))
        //    {
        //      ROS_ERROR("Programming Error");
        //      return false;
        //    }
    }

    uchar* dataImg = rawHSV.data;
    uchar* dataFieldImg = fieldMask.data;
    for (size_t i = 0; i < allLines.size(); i++) {
        double greenVoter = 0;
        int vote_for_double_up = 0;
        int vote_for_double_down = 0;
        int vote_for_color_up = 0;
        int vote_for_color_down = 0;
        int not_valid_point = 0;

        // 遍历每个等分点
        for (size_t j = 0; j < COUNT_MID_P; j++) {
            int idx = i * COUNT_MID_P + j;
            int cur_upArraySize = 0;
            int cur_downArraySize = 0;
            cv::LineIterator itUp(lineBinary, midds[i][j], PerpendicularLS[idx].P1, 8);
            cv::LineIterator itDown(lineBinary, midds[i][j], PerpendicularLS[idx].P2, 8);
            cv::Point cenP = queryPoint[idx * 3];
            double upDistance = 0.0;
            double downDistance = 0.0;
            if (aprxDist) {
                cv::Point upP = queryPoint[idx * 3 + 1];
                cv::Point downP = queryPoint[idx * 3 + 2];

                // 获得向上向下点的距离
                upDistance = GetDistance(cenP, upP);
                downDistance = GetDistance(cenP, downP);

                // 获得向上向下实际点的距离
                // cv::Point2f cenPR = realQueryPoint[idx * 3];
                // cv::Point2f upPR = realQueryPoint[idx * 3 + 1];
                // cv::Point2f downPR = realQueryPoint[idx * 3 + 2];
                // double upDistanceR = GetDistance(cenPR, upPR);
                // double downDistanceR = GetDistance(cenPR, downPR);
                // 实际坐标系中的两个点的距离 × 图片中这两个点的距离 =
                // 要求距离line_width_check下的像素距离 ？ 难道实际坐标的单位是米？
                cur_upArraySize = itUp.count;
                cur_downArraySize = itDown.count;
                // TODO(yyj) implement IPM
                // cur_upArraySize = min(itUp.count, (int)((LINE_WIDTH_CHECK / upDistanceR) * upDistance));
                // cur_downArraySize = min(itDown.count, (int)((LINE_WIDTH_CHECK / downDistanceR) * downDistance));
            } else {
                cur_upArraySize = bothEndSize[idx * 2];
                cur_downArraySize = bothEndSize[idx * 2 + 1];
                upDistance = cur_upArraySize;
                downDistance = cur_downArraySize;
            }

            cur_upArraySize = std::max(jumpMin, cur_upArraySize);
            cur_downArraySize = std::max(jumpMin, cur_downArraySize);

            if (upDistance > 0.1 && downDistance > 0.1 && itUp.count >= jumpMin && itDown.count >= jumpMin) {
                cv::Point end_up, end_down;
                bool firstColor = parameters.line.colorVUse;
                bool firstGreen = parameters.line.greenVUse;
                bool firstDouble = parameters.line.doubleVUse;

                int startDouble = static_cast<int>(cur_upArraySize * parameters.line.doubleVStart);
                int endDouble = static_cast<int>(cur_upArraySize * parameters.line.doubleVEnd);
                int startGreen = static_cast<int>(cur_upArraySize * parameters.line.greenVStart);
                int endGreen = static_cast<int>(cur_upArraySize * parameters.line.greenVEnd);
                int startColor = static_cast<int>(cur_upArraySize * parameters.line.colorVStart);
                int endColor = static_cast<int>(cur_upArraySize * parameters.line.colorVEnd);

                for (int k = 0; k < cur_upArraySize; k++, ++itUp) {
                    cv::Point p = itUp.pos();
                    end_up = p;
                    if (k > 0) {
                        // 行数×1920
                        int yPixel = p.y * parameters.camera.width;
                        // 指向该像素的指针
                        uchar* pixel = dataImg + (yPixel + p.x) * 3;
                        uchar h = pixel[0];
                        uchar s = pixel[1];
                        uchar v = pixel[2];
                        uchar* pixelF = dataFieldImg + yPixel + p.x;
                        // 在每一小段垂直线段中，
                        // 一旦线段中的某个像素在lineBinary（canny算子结果中高响应）中是255，即是边缘线那么firstDouble
                        //= false;
                        if (firstDouble && k >= startDouble && k <= endDouble) {
                            if (*(*itUp) > 254) {
                                vote_for_double_up++;
                                firstDouble = false;
                            }
                        }
                        // 在每一小段垂直线段中，
                        // 一旦线段中的某个像素符合白线的hsv范围，就对vote_for_color_up加1，
                        // firstColor = false;
                        if (firstColor && k >= startColor && k <= endColor) {
                            if (h >= parameters.line.h0 && h <= parameters.line.h1 && s >= parameters.line.s0 && s <= parameters.line.s1 && v >= parameters.line.v0 && v <= parameters.line.v1) {
                                vote_for_color_up++;
                                firstColor = false;
                            }
                        }
                        // 在每一小段垂直线段中，
                        // 一旦线段中的某个像素符合白线的hsv范围，就对greenVoter加0.5，
                        // firstColor = false;
                        if (firstGreen && k >= startGreen && k <= endGreen) {
                            if (*pixelF > 254) {
                                greenVoter += 0.5;
                                firstGreen = false;
                            }
                        }
                    }
                    //三个条件都符合了就不用继续做了
                    if (!parameters.line.showVote && !firstColor && !firstGreen && !firstDouble) {
                        break;
                    }
                }
                firstColor = parameters.line.colorVUse;
                firstGreen = parameters.line.greenVUse;
                firstDouble = parameters.line.doubleVUse;

                // 然后在itdowm中继续做这些事情
                for (int k = 0; k < cur_downArraySize; k++, ++itDown) {
                    cv::Point p = itDown.pos();
                    end_down = p;
                    if (k > 0) {
                        int ypixel = p.y * parameters.camera.width;
                        uchar* pixel = dataImg + (ypixel + p.x) * 3;
                        uchar h = pixel[0];
                        uchar s = pixel[1];
                        uchar v = pixel[2];
                        uchar* pixelF = dataFieldImg + ypixel + p.x;
                        if (firstDouble && k >= startDouble && k <= endDouble) {
                            if (*(*itDown) > 254) {
                                vote_for_double_down++;
                                firstDouble = false;
                            }
                        }
                        if (firstColor && k >= startColor && k <= endColor) {
                            if (h >= parameters.line.h0 && h <= parameters.line.h1 && s >= parameters.line.s0 && s <= parameters.line.s1 && v >= parameters.line.v0 && v <= parameters.line.v1) {
                                vote_for_color_down++;
                                firstColor = false;
                            }
                        }
                        if (firstGreen && k >= startGreen && k <= endGreen) {
                            if (*pixelF > 254) {
                                greenVoter += 0.5;
                                firstGreen = false;
                            }
                        }
                    }
                    if (!parameters.line.showVote && !firstColor && !firstGreen && !firstDouble) {
                        break;
                    }
                }
                // 画出投票的结果，应该是好多小短线？
                if (showGui && parameters.line.showVote) {
                    cv::line(guiImg, end_up, end_down, yellowColor(), 1);
                    cv::circle(guiImg, end_up, 1, blueColor(), 1);
                    cv::circle(guiImg, end_down, 1, redColor(), 1);
                }
            } else {
                not_valid_point++;
            }
        }
        // 无效点数少于一半
        // bool checkIsValid = ((not_valid_point * 2) < COUNT_MID_P);
        bool checkIsValid = (COUNT_MID_P > (not_valid_point * 2));
        if (checkIsValid) {
            bool greenOk = true;
            bool upDoubleOk = true;
            bool downDoubleOk = true;
            bool upColorOk = true;
            bool downColorOk = true;
            double probability = 1;
            double totalVP = (COUNT_MID_P - not_valid_point);
            if (parameters.line.greenVUse) {
                int MIN_LINE_GREEN_VOTE = static_cast<int>((parameters.line.greenVote / 100.) * totalVP);
                greenOk = (greenVoter >= MIN_LINE_GREEN_VOTE);
                probability *= (greenVoter / totalVP);
            }
            if (parameters.line.doubleVUse) {
                int MIN_LINE_DOUBLE_VOTE = static_cast<int>((parameters.line.doubleVote / 100.) * totalVP);
                upDoubleOk = (vote_for_double_up >= MIN_LINE_DOUBLE_VOTE);
                downDoubleOk = (vote_for_double_down >= MIN_LINE_DOUBLE_VOTE);
                probability *= (std::max(vote_for_double_up, vote_for_double_down) / totalVP);
            }
            if (parameters.line.colorVUse) {
                int MIN_LINE_COLOR_VOTE = static_cast<int>((parameters.line.colorVote / 100.) * totalVP);
                upColorOk = (vote_for_color_up >= MIN_LINE_COLOR_VOTE);
                downColorOk = (vote_for_color_down >= MIN_LINE_COLOR_VOTE);
                probability *= (std::max(vote_for_color_up, vote_for_color_down) / totalVP);
            }
            // white line
            bool colorOK = (downColorOk || upColorOk);
            // canny
            bool doubleOK = (downDoubleOk || upDoubleOk);
            if (greenOk && colorOK && doubleOK) {
                allLines[i].SetProbability(probability);
                resLines.push_back(allLines[i]);
            } else {
                if (parameters.line.showAllLine && showGui) {
                    cv::line(guiImg, allLines[i].P1, allLines[i].P2, redColor(), 1, 8);
                }
            }
        } else {
            if (parameters.line.showAllLine && showGui) {
                cv::line(guiImg, allLines[i].P1, allLines[i].P2, redMeloColor(), 1, 8);
            }
        }
    }
    return resLines.size() > 0;
}
} // namespace dvision
