// Created on: May 13, 2017
//     Author: Yujie Yang <meetyyj@gmail.com>

#pragma once
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>

namespace dvision {
class IPM
{

  private:
    Eigen::MatrixXd mMat;

  public:
    /**
     * We first calculate 4 corner points'(in undistorted image) location at real(in field) coordinate.
     * Then using OpenCV's function findHomography(InputArray srcPoints, InputArray dstPoints) to get the homography mat.
     * @param cameraLocation            : input camera location when update         (unit:cm)
     * @param cameraOrientation         : input camera orientation when update      (unit:radius)
     * @param homoFor                   : output the forward homography mat         (used for OpenCV perspectiveTransform)
     * @param homoBack                  : output the back homography mat            (used for OpenCV perspectiveTransform)
     */
    bool initGetHomography(const Eigen::MatrixXd& extrinsic, cv::Mat& homoFor, cv::Mat& homoBack);

    /**
     * http://docs.opencv.org/3.1.0/d9/d0c/group__calib3d.html
     * We know Point2d(u, v) in undistorted image.
     * And fx, fy are camera focus.
     * Take care that undistorted image's offset cx' & cy' has changed to
     * cx' = (params.camera.widthUnDistortion - params.camera.width) / 2 + params.camera.cx
     * cy' = (params.camera.heightUnDistortion - params.camera.height) / 2 + params.camera.cy
     * Mat m is mMat calculated in initGetHomography using camera's location and orientation.
     * Finally X and Y is the Point(u, v)'s projection in real coordinate.
     * [u    [fx   0   cx'    [m00  m01  m02  m03    [X
     *  v  =   0  fy   cy'  *  m10  m11  m12  m13  *  Y
     *  1]     0   0     1]    m20  m21  m22  m23]    0
     *                                                1]
     * The function implements the projection process described above.
     * @param contour                   : input undistorted point                   (unit:pixel)
     * @param resContour                : output real coordinate point              (unit:cm)
     */
    bool calculatePoints(std::vector<cv::Point2f>& contour, std::vector<cv::Point2f>& resContour);
};
} // namespace dvision
