#include "dvision/ipm.hpp"
#include "dvision/parameters.hpp"
using namespace cv;
using namespace std;
using namespace Eigen;

namespace dvision {
bool

IPM::initGetHomography(const MatrixXd& extrinsic, Mat& homoFor, Mat& homoBack)
{
    mMat = extrinsic;
    //    cameraLocation.x /= 0.01;
    //    cameraLocation.y /= 0.01;
    //    cameraLocation.z /= 0.01;
    //
    //    MatrixXd tra(4, 4);
    //    tra << 1, 0, 0, -cameraLocation.x, //
    //      /**/ 0, 1, 0, -cameraLocation.y, //
    //      /**/ 0, 0, 1, -cameraLocation.z, //
    //      /**/ 0, 0, 0, 1.000000000000000; //
    //
    //    double sin_alpha = sin(cameraOrientation.z);
    //    double cos_alpha = cos(cameraOrientation.z);
    //
    //    double sin_beta = sin(cameraOrientation.y);
    //    double cos_beta = cos(cameraOrientation.y);
    //
    //    double sin_gama = sin(cameraOrientation.x);
    //    double cos_gama = cos(cameraOrientation.x);
    //
    //    MatrixXd rotAroundZ(4, 4);
    //    rotAroundZ << cos_alpha, sin_alpha, 0, 0,  //
    //      /*********/ -sin_alpha, cos_alpha, 0, 0, //
    //      /*********/ 0.0000000, 0.0000000, 1, 0,  //
    //      /*********/ 0.0000000, 0.0000000, 0, 1;  //
    //
    //    MatrixXd rotAroundY(4, 4);
    //    rotAroundY << cos_beta, 0, -sin_beta, 0, //
    //      /*********/ 0.000000, 1, 0.000000, 0,  //
    //      /*********/ sin_beta, 0, cos_beta, 0,  //
    //      /*********/ 0.000000, 0, 0.000000, 1;  //
    //
    //    MatrixXd rotAroundX(4, 4);
    //    rotAroundX << 1, 0.000000, 0.000000, 0,  //
    //      /*********/ 0, cos_gama, sin_gama, 0,  //
    //      /*********/ 0, -sin_gama, cos_gama, 0, //
    //      /*********/ 0, 0.000000, 0.000000, 1;  //
    //
    //    mMat = rotAroundZ * rotAroundX * rotAroundY * tra;
    //    cout << "mMat:" << endl;
    //    cout << mMat << endl;

    ROS_FATAL("Not implemented");
    return true;

    vector<Point2f> cornerUndistortImgPoint;
    cornerUndistortImgPoint.push_back(Point2f(0, 0));
    cornerUndistortImgPoint.push_back(Point2f(0, 1));
    cornerUndistortImgPoint.push_back(Point2f(1, 0));
    cornerUndistortImgPoint.push_back(Point2f(1, 1));

    vector<Point2f> cornerRealPoint;
    if (calculatePoints(cornerUndistortImgPoint, cornerRealPoint)) {
        homoFor = findHomography(cornerUndistortImgPoint, cornerRealPoint);
        homoBack = findHomography(cornerRealPoint, cornerUndistortImgPoint);
    } else {
        return false;
    }
    return true;
}

// fuck

bool
IPM::calculatePoints(vector<Point2f>& contour, vector<Point2f>& resContour)
{
    resContour.resize(contour.size());
    for (uint32_t i = 0; i < contour.size(); ++i) {
        double x = (contour[i].x - parameters.camera.undistCx) / parameters.camera.fx;
        double y = (contour[i].y - parameters.camera.undistCy) / parameters.camera.fy;

        MatrixXd solve_A(2, 2);
        solve_A << mMat(0, 0) - mMat(2, 0) * x, mMat(0, 1) - mMat(2, 1) * x, mMat(1, 0) - mMat(2, 0) * y, mMat(1, 1) - mMat(2, 1) * y;

        // Check if solve_A doesn't has a inverse matrix, means that solve_A is linear correlation
        if (solve_A(0, 0) * solve_A(1, 1) == solve_A(0, 1) * solve_A(1, 0)) {
            cout << " fuck " << endl;
            return false;
        }
        MatrixXd solve_B(2, 1);
        solve_B << mMat(2, 3) * x - mMat(0, 3), mMat(2, 3) * y - mMat(1, 3);
        MatrixXd inverse2ground = solve_A.inverse() * solve_B;
        double rx = -inverse2ground(0, 0);
        double ry = inverse2ground(1, 0);
        resContour[i].x = rx;
        resContour[i].y = ry;
    }

    return true;
}
} // namespace dvision
