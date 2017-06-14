// Created on: May 13, 2017
//     Author: Yujie Yang <meetyyj@gmail.com>

#pragma once
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>
#include "dvision/parameters.hpp"
#include <cmath>
using dvision::parameters;
using Eigen::MatrixXd;
using std::cos;
using std::sin;
namespace dvision {
class IPM
{

  private:
    Eigen::MatrixXd mMat;
    Eigen::MatrixXd m_extrinsic;

  public:

    // todo(MWX), calc_extrinsic according to servo angles

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


    // added by mwx 2017.6.14
    // todo, put this into utils
    inline MatrixXd rotateX(double r) {
        MatrixXd R(4, 4);
        R << 1, 0, 0, 0,
             0, cos(r), sin(r), 0,
             0, -sin(r), cos(r), 0,
             0,  0, 0, 1;
        return R;
    }

    inline MatrixXd rotateY(double r) {
        MatrixXd R(4, 4);
        R << cos(r), 0, sin(r), 0,
              0, 1, 0, 0,
            -sin(r), 0, cos(r), 0,
              0, 0, 0, 1;
    }

    inline MatrixXd rotateZ(double r) {
        MatrixXd R(4, 4);
        R << cos(r), sin(r), 0, 0,
            -sin(r), cos(r), 0, 0,
                  0,      0, 1, 0,
                  0,      0, 0, 1;
        return R;
    }

    inline MatrixXd dtranslate(double x, double y, double z) {
        MatrixXd T(4, 4);
        T << 1, 0, 0, x,
             0, 1, 0, y,
             0, 0, 1, z,
             0, 0, 0, 1;
        return T;
    }

    inline void calc_extrinsic(int pitchRad, int yawRad) {
        auto& extrinsic_para = parameters.camera.extrinsic_para;
        auto Xw2p = extrinsic_para[0];
        auto Yw2p = extrinsic_para[1];
        auto Zw2p = extrinsic_para[2];

        auto RXw2p = extrinsic_para[3];
        auto RYw2p = extrinsic_para[4];
        auto RZw2p = extrinsic_para[5];
        auto Xp2c = extrinsic_para[6];
        auto Yp2c = extrinsic_para[7];
        auto Zp2c = extrinsic_para[8];

        auto RXp2c = extrinsic_para[9];
        auto RYp2c = extrinsic_para[10];
        auto RZp2c = extrinsic_para[11];

        auto scaleYaw = extrinsic_para[12];
        auto scalePitch = extrinsic_para[13];
        auto biasYaw = extrinsic_para[14];
        auto biasPitch = extrinsic_para[15];

        double pitch = (pitchRad + biasPitch) * scalePitch;
        double yaw = (yawRad + biasYaw) * scaleYaw;

        MatrixXd w2p(4, 4);
        w2p = rotateZ(RZw2p)
            * rotateY(RYw2p)
            * rotateX(RXw2p)
            * dtranslate(-Xw2p, -Yw2p, -Zw2p);

        MatrixXd p2c(4,4);
        p2c = rotateZ(RZp2c)
            * rotateY(RYp2c)
            * rotateX(RXp2c)
            * dtranslate(-Xp2c, -Yp2c, -Zp2c)
            * rotateY(-pitch)
            * rotateZ(yaw);
    }
};
} // namespace dvision
