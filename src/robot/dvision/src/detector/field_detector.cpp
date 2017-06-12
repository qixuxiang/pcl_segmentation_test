/**
 * @Author: Yusu Pan <corenel>
 * @Date:   2017-06-02T19:46:50+08:00
 * @Email:  xxdsox@gmail.com
 * @Project: humanoid
 * @Filename: field_detector.cpp
 * @Last modified by:   corenel
 * @Last modified time: 2017-06-02T19:48:14+08:00
 * @Copyright: ZJUDancer
 */

#include "dvision/field_detector.hpp"
#include "dvision/parameters.hpp"

namespace dvision {
FieldDetector::FieldDetector()
{
}

bool
FieldDetector::Init()
{
    // TODO(corenel) init BodyMaskContourInverted
    //      cv::FileStorage fr(parameters.configPath+"BodyMask.yml",
    //                         cv::FileStorage::READ);
    //      cv::FileNode fileNode = fr["IGUS"];
    //      read(fileNode, BodyMaskContourInverted);
    //      fr.release();
    //      if (BodyMaskContourInverted.size() < 6)
    //      {
    //        ROS_ERROR("Create or modify BodyMask.yml!");
    //      }
    ROS_INFO("FieldDetector Init() started");
    return true;
}

FieldDetector::~FieldDetector()
{
}

bool
FieldDetector::GetPoints(cv::Mat& binaryFrame, std::vector<cv::Point>& resPoints, std::vector<std::vector<cv::Point>>& allFieldContours)
{
    if (parameters.field.erode > 0) {
        erode(binaryFrame, binaryFrame, cv::Mat(), cv::Point(-1, -1), parameters.field.erode);
    }
    if (parameters.field.dilate > 0) {
        dilate(binaryFrame, binaryFrame, cv::Mat(), cv::Point(-1, -1), parameters.field.dilate);
    }
    if (parameters.field.erode2 > 0) {
        erode(binaryFrame, binaryFrame, cv::Mat(), cv::Point(-1, -1), parameters.field.erode2);
    }
    if (parameters.field.dilate2 > 0) {
        dilate(binaryFrame, binaryFrame, cv::Mat(), cv::Point(-1, -1), parameters.field.dilate2);
    }
    cv::findContours(binaryFrame.clone(), allFieldContours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    resPoints.reserve(parameters.field.maxContourCount);
    std::sort(allFieldContours.begin(), allFieldContours.end(), SortFuncDescending);
    bool ret = false;
    int totalResult = 0;
    for (size_t i = 0; i < allFieldContours.size(); i++) {
        if (totalResult >= parameters.field.maxContourCount) {
            return ret;
        }
        cv::Rect rec = cv::boundingRect(allFieldContours[i]);
        double area = cv::contourArea(allFieldContours[i]);
        if (std::abs(parameters.camera.height - Bottom(rec)) <= parameters.field.maxDownDiffPixel && area >= parameters.field.minArea) {
            cv::approxPolyDP(allFieldContours[i], allFieldContours[i], cv::arcLength(allFieldContours[i], true) * 0.003, true);
            for (size_t pIt = 0; pIt < allFieldContours[i].size(); pIt++) {
                resPoints.push_back(allFieldContours[i][pIt]);
            }
            ret = true;
        }
        totalResult++;
    }
    return ret;
}

void
FieldDetector::FindInField(const cv::Mat& srcHsvImg, const cv::Mat& templateGrayImg, cv::Mat* dstGrayImgs, HSVRange* ranges, bool* inTemplate, int size)
{
    const int srcSize = srcHsvImg.rows * srcHsvImg.cols;
    int* indexs = new int[4];
    for (int k = 0; k < size; k++) {
        indexs[k] = 0;
    }
    uchar* srcHsvImg_D = srcHsvImg.data;
    uchar* templateGrayImg_D = templateGrayImg.data;

    for (int i = 0; i < srcSize; i++) {
        ushort h = srcHsvImg_D[0], s = srcHsvImg_D[1], v = srcHsvImg_D[2];
        if (templateGrayImg_D[0] >= 254) {
            for (int k = 0; k < size; k++) {
                if (h >= ranges[k].h0 && h <= ranges[k].h1 && s >= ranges[k].s0 && s <= ranges[k].s1 && v >= ranges[k].v0 && v <= ranges[k].v1) {
                    dstGrayImgs[k].data[indexs[k]] = 255;
                } else {
                    dstGrayImgs[k].data[indexs[k]] = 0;
                }
            }
        } else {
            for (int k = 0; k < size; k++) {
                if (inTemplate[k])
                    continue;
                if (h >= ranges[k].h0 && h <= ranges[k].h1 && s >= ranges[k].s0 && s <= ranges[k].s1 && v >= ranges[k].v0 && v <= ranges[k].v1) {
                    dstGrayImgs[k].data[indexs[k]] = 255;
                } else {
                    dstGrayImgs[k].data[indexs[k]] = 0;
                }
            }
        }
        templateGrayImg_D += 1;
        srcHsvImg_D += 3;
        for (int k = 0; k < size; k++) {
            indexs[k]++;
        }
    }
    delete[] indexs;
}

std::vector<cv::Point>
FieldDetector::getBodyMaskContourInRaw(float rot)
{
    std::vector<cv::Point> res;
    if (BodyMaskContourInverted.size() < 6)
        return res;
    for (size_t i = 0; i < BodyMaskContourInverted.size(); i++) {
        cv::Point rotated = RotateAroundPoint(BodyMaskContourInverted[i], rot);
        cv::Point p(static_cast<int>(rotated.x + parameters.camera.width / 2.), abs(rotated.y - 480));
        if (p.inside(cv::Rect(0, 0, parameters.camera.width, parameters.camera.height))) {
            if (res.size() == 0) {
                res.push_back(cv::Point(p.x, parameters.camera.height));
            }
            res.push_back(p);
        }
    }
    if (res.size() > 0) {
        int xlast = res[res.size() - 1].x;
        res.push_back(cv::Point(xlast, parameters.camera.height));
    }
    return res;
}

} // namespace dvision
