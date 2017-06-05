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
GoalDetector::GetPosts(cv::Mat& cannyImg,
                       cv::Mat& rawHSV,
                       cv::Mat& gray,
                       const cv::Mat& binaryFrame,
                       Projection& projection,
                       const std::vector<cv::Point>& fieldHull,
                       std::vector<LineSegment>& resLines,
                       std::vector<LineSegment>& alllLines,
                       std::vector<cv::Point2f>& goalPosition,
                       const bool& SHOWGUI,
                       cv::Mat& guiImg)
{
    // never use
    // if (binaryFrame.empty()) {
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
    HoughLinesP(cannyImg, linesP, 1, M_PI / 45, 10, MinLineLength, 10);
    std::vector<LineSegment> allVerLines;
    for (size_t i = 0; i < linesP.size(); i++) {
        // lP (x_1, y_1, x_2, y_2)
        cv::Vec4i lP = linesP[i];
        LineSegment tmpLine(cv::Point2d(lP[0], lP[1]), cv::Point2d(lP[2], lP[3]));
        // std::vector<cv::Point> goalPostPoints;
        // 与竖直线夹角小于15度
        if (tmpLine.GetAbsMinAngleDegree(LineSegment(cv::Point(0, 0), cv::Point(0, 100))) < 15) {
            // 画出所有Hough到的竖直线
            // cv::line(guiImg, tmpLine.P1, tmpLine.P2, darkOrangeColor(), 1);

            double leftAvg = 0;
            double rightAvg = 0;

            // 取2^5+1=33个等分点
            std::vector<cv::Point2d> midds = tmpLine.GetMidPoints(5);
            int maxMidPoint = midds.size();
            int vote_for_doubleLeft = 0;
            int vote_for_doubleRight = 0;

            // 取下端点
            cv::Point down = (tmpLine.P1.y < tmpLine.P2.y) ? tmpLine.P2 : tmpLine.P1;
            double jumpDouble = parameters.goal.jumpMax;

            if (!checkDownPointDistance(down, jumpDouble, projection, fieldHull)) {
                continue;
            }

            for (size_t j = 0; j < midds.size(); j++) {
                if (!voteGoalPostPoint(tmpLine, midds[j], jumpDouble, SHOWGUI, guiImg, rawHSV, cannyImg, leftAvg, rightAvg, vote_for_doubleLeft, vote_for_doubleRight)) {
                    continue;
                }
            }

            bool leftOK = (vote_for_doubleLeft / static_cast<float>(maxMidPoint)) * 100. > parameters.goal.doubleVote;

            bool rightOK = (vote_for_doubleRight / static_cast<float>(maxMidPoint)) * 100. > parameters.goal.doubleVote;

            if (leftOK || rightOK) {
                {
                    LineSegment tmpLineChanged = tmpLine;

                    // 球门柱下端点下探避免
                    // {
                    //   cv::Point validDown =
                    //       (tmpLine.P1.y < tmpLine.P2.y) ? tmpLine.P2 : tmpLine.P1;
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
                    //   if (tmpLineChanged.P1.y < tmpLineChanged.P2.y) {
                    //     tmpLineChanged.P2 = validDown;
                    //   } else {
                    //     tmpLineChanged.P1 = validDown;
                    //   }
                    // }

                    // 平移，将球门柱左边缘线或右边缘线向中间平移
                    if (leftOK) {
                        int amount = abs(leftAvg / vote_for_doubleLeft) / 2.;
                        tmpLineChanged.P1.x -= amount;
                        tmpLineChanged.P2.x -= amount;
                    } else if (rightOK) {
                        int amount = abs(rightAvg / vote_for_doubleRight) / 2.;
                        tmpLineChanged.P1.x += amount;
                        tmpLineChanged.P2.x += amount;
                    }
                    tmpLineChanged.Clip(rec);

                    allVerLines.push_back(tmpLineChanged);
                }
            }
        }
    }

    // 线段融合
    std::vector<LineSegment> allVerLines2;
    MergeLinesMax(allVerLines, 30, DistanceToMerge, allVerLines2, rec);

    for (size_t i = 0; i < allVerLines2.size(); i++) {
        LineSegment tmpLine = allVerLines2[i];
        cv::Point up = (tmpLine.P1.y > tmpLine.P2.y) ? tmpLine.P2 : tmpLine.P1;
        cv::Point down = (tmpLine.P1.y < tmpLine.P2.y) ? tmpLine.P2 : tmpLine.P1;

        double verLen = tmpLine.GetLength();
        // 线片段上端点须在场地凸包外一定距离
        if (pointPolygonTest(fieldHull, up, true) > MinNearFieldUpPoint) {
            // cout << "Goal: up in field" << endl;
            continue;
        }

        cv::Point2f downReal;
        if (!projection.getOnRealCoordinate(down, downReal)) {
            // ROS_ERROR("Erorr in programming!");
            return false;
        }

        // 根本不知道在干什么
        if (!checkDistance_Box(downReal, verLen)) {
            // cout << "Goal: check box error" << endl;
            continue;
        }
        // 线片段下端点须在场地凸包内
        double downDistance2Field = pointPolygonTest(fieldHull, down, true);
        if (downDistance2Field < MaxOutField) {
            // cout << "Goal: down not in field" << endl;
            continue;
        }
        // 线片段与机器人距离大于5m亦舍弃
        if (GetDistance(downReal) > 750) {
            // cout << "Goal: too far from robot" << endl;
            continue;
        }

        // 对于符合上述条件的goal post，从下端点开始进行下探
        // 直到没有高响应的点
        int cntInvalidPoints = 0;
        int cntTotalExtPoints = 0;

        // invalid point出现五个或者总计延长点超过15个
        // 则终止延长
        while (cntInvalidPoints <= 5 && cntTotalExtPoints <= 15) {
            // 取tmpLine向下的延长线上的一点
            cv::Point2d downExtensionPoint = tmpLine.ExtensionPointDown(2);

            // 检查延长点与机器人距离，并获取jumpDouble
            double jumpDouble = parameters.goal.jumpMax;
            if (!checkDownPointDistance(downExtensionPoint, jumpDouble, projection, fieldHull)) {
                continue;
            }
            // 检查高响应
            double leftAvg = 0;
            double rightAvg = 0;
            int vote_for_doubleLeft = 0;
            int vote_for_doubleRight = 0;
            if (!voteGoalPostPoint(tmpLine, downExtensionPoint, jumpDouble, SHOWGUI, guiImg, rawHSV, cannyImg, leftAvg, rightAvg, vote_for_doubleLeft, vote_for_doubleRight)) {
                continue;
            }
            bool leftOK = (vote_for_doubleLeft / 1.) * 100. > parameters.goal.doubleVote / 2;
            bool rightOK = (vote_for_doubleRight / 1.) * 100. > parameters.goal.doubleVote / 2;

            // 若左右侧出现足够多的高响应点，则该延长点valid
            if (leftOK || rightOK) {
                tmpLine.SetDownPoint(downExtensionPoint);
                // cout << "Expand to (" << downExtensionPoint.x << ","
                //      << downExtensionPoint.y << ")" << endl;
            } else {
                // cout << "Invalid (" << downExtensionPoint.x << ","
                //      << downExtensionPoint.y << ")" << endl;
                cntInvalidPoints++;
            }
            cntTotalExtPoints++;
        }

        // 获取当前的下端点以及其对应的机器人坐标系中的点
        down = tmpLine.GetDownPoint();
        if (!projection.getOnRealCoordinate(down, downReal)) {
            // ROS_ERROR("Erorr in programming!");
            return false;
        }

        // 令人感动球门柱下端点加了进去
        goalPosition.push_back(downReal);
        // 顺便加了整个球门柱的线片段
        resLines.push_back(LineSegment(down, up));
    }

    if (SHOWGUI && parameters.goal.showVote) {
        for (size_t i = 0; i < allVerLines2.size(); i++) {
            cv::line(guiImg, allVerLines2[i].P1, allVerLines2[i].P2, darkOrangeColor(), 1);
        }
    }
    if (SHOWGUI && parameters.goal.showResult) {
        for (size_t i = 0; i < resLines.size(); i++) {
            cv::line(guiImg, resLines[i].P1, resLines[i].P2, blueColor(), 2);
        }
    }
    return resLines.size() > 0;
}

bool
GoalDetector::checkDistance_Box(cv::Point2f downPointInReal, double length)
{
    LineSegment lowerBound(cv::Point2f(parameters.goal.NearestDistance, parameters.goal.NearMinLen), cv::Point2f(parameters.goal.FurthestDistance, parameters.goal.FarMinLen));
    LineSegment higherBound(cv::Point2f(parameters.goal.NearestDistance, parameters.goal.NearMaxLen), cv::Point2f(parameters.goal.FurthestDistance, parameters.goal.FarMaxLen));
    LinearBoundaryChecker checker(lowerBound, higherBound);

    double distanceToRobot = GetDistance(downPointInReal);

    return checker.CheckInside(distanceToRobot, length);
}

bool
GoalDetector::checkDownPointDistance(const cv::Point& down, double& jumpDouble, Projection& projection, std::vector<cv::Point> fieldHull)
{
    // 该端点在机器人坐标系中的位置
    cv::Point2f downReal;
    if (!projection.getOnRealCoordinate(down, downReal)) {
        // ROS_ERROR("Erorr in programming!");
        return false;
    }
    double downDistance2Field = pointPolygonTest(fieldHull, down, true);
    if (downDistance2Field < parameters.goal.MaxOutField)
        return false;
    // 根据down点与机器人的距离来选择不同的jump值
    double distance = GetDistance(downReal);
    if (distance < 200) {
        jumpDouble = 40 * 1920 / 640;
    } else if (distance >= 200 && distance < 300) {
        jumpDouble = 23 * 1920 / 640;
    } else {
        jumpDouble = 15 * 1920 / 640;
    }

    return true;
}

bool
GoalDetector::voteGoalPostPoint(LineSegment& tmpLine,
                                const cv::Point2d& point,
                                const double& jumpDouble,
                                const bool& SHOWGUI,
                                cv::Mat& guiImg,
                                const cv::Mat& rawHSV,
                                const cv::Mat& cannyImg,
                                double& leftAvg,
                                double& rightAvg,
                                int& vote_for_doubleLeft,
                                int& vote_for_doubleRight)
{
    // 该点时候是合格的goalPostPoint的flag
    // bool validGoalPostPoint = false;
    // 从某个等分点上取垂线段
    LineSegment tocheck = tmpLine.PerpendicularLineSegment(jumpDouble, point);

    // 左右垂线段片段
    cv::Point left = (tocheck.P1.x < tocheck.P2.x) ? tocheck.P1 : tocheck.P2;
    cv::Point right = (tocheck.P1.x < tocheck.P2.x) ? tocheck.P2 : tocheck.P1;
    cv::LineIterator itLeft(cannyImg, point, left, 8);
    cv::LineIterator itRight(cannyImg, point, right, 8);
    cv::LineIterator itHSVLeft(rawHSV, point, left, 8);
    cv::LineIterator itHSVRight(rawHSV, point, right, 8);

    int safeToShow = 0;
    if (tocheck.P1.x >= 0 && tocheck.P1.y >= 0 && tocheck.P1.x < parameters.camera.width && tocheck.P1.y < parameters.camera.height) {
        safeToShow++;
    }
    if (tocheck.P2.x >= 0 && tocheck.P2.y >= 0 && tocheck.P2.x < parameters.camera.width && tocheck.P2.y < parameters.camera.height) {
        safeToShow++;
    }

    for (int k = 0; k < itLeft.count; k++, ++itLeft, ++itHSVLeft) {
        if (k < 2)
            continue;
        uchar val = *(*itLeft);
        cv::Vec3b hsvC = (cv::Vec3b)*itHSVLeft;

        // Canny高响应（如球门柱左边缘线上的某个等分点的垂线段碰到了右边缘线）
        // 且左右边缘线像素距离大于minDoubleLength
        if (val > 0 && k > parameters.goal.minDoubleLength) {
            // 在guiImg上画出来
            if (safeToShow >= 2 && SHOWGUI && parameters.goal.showVote) {
                cv::line(guiImg, point, itHSVLeft.pos(), redColor(), 1);
            }
            // 用以之后计算平移距离
            leftAvg += k;
            // 投票也有我一份
            vote_for_doubleLeft++;
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
    for (int k = 0; k < itRight.count; k++, ++itRight, ++itHSVRight) {
        if (k < 2)
            continue;
        uchar val = *(*itRight);
        cv::Vec3b hsvC = (cv::Vec3b)*itHSVRight;

        if (val > 0 && k > parameters.goal.minDoubleLength) {
            if (safeToShow >= 2 && SHOWGUI && parameters.goal.showVote) {
                cv::line(guiImg, point, itHSVRight.pos(), redColor(), 1);
            }
            rightAvg += k;
            vote_for_doubleRight++;
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
