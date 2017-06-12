#include "dvision/distortionModel.hpp"
#include "dvision/parameters.hpp"

using namespace std;
using namespace cv;

namespace dvision {
DistortionModel::DistortionModel()
{
}

void
DistortionModel::init()
{
    int W = parameters.camera.width;
    int H = parameters.camera.height;

    vector<Point> center, resCenter;
    center.push_back(Point(W / 2, H / 2));
    undistort_slow(center, resCenter);
    cout << "Undistort center: " << resCenter << endl;

    vector<Point> src, res;

    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x)
            src.push_back(Point(x, y));

    undistort_slow(src, res);

    int maxH = -1;
    int maxW = -1;

    for (uint32_t i = 0; i < res.size(); ++i) {
        int tmpW = max(maxW, abs(res[i].x - resCenter[0].x));
        int tmpH = max(maxH, abs(res[i].y - resCenter[0].y));
        if (tmpW > 700 || tmpH > 700)
            continue;
        maxW = tmpW;
        maxH = tmpH;
    }

    parameters.camera.undistWidth = maxW * 2 + 1;
    parameters.camera.undistHeight = maxH * 2 + 1;
    parameters.camera.undistImageSize = Size(maxW * 2 + 1, maxH * 2 + 1);
    cout << "Undistorted image size:" << parameters.camera.undistImageSize << endl;

    int offsetX = maxW - resCenter[0].x;
    int offsetY = maxH - resCenter[0].y;

    //    int offsetX = (m_undistImageSize.width - W) / 2.;
    //    int offsetY = (m_undistImageSize.height - H) / 2.;

    parameters.camera.undistCameraMatrix = parameters.camera.cameraMatrix.clone();
    parameters.camera.undistCameraMatrix.at<double>(0, 2) += offsetX;
    parameters.camera.undistCameraMatrix.at<double>(1, 2) += offsetY;
    parameters.camera.undistCx = parameters.camera.undistCameraMatrix.at<double>(0, 2);
    parameters.camera.undistCy = parameters.camera.undistCameraMatrix.at<double>(1, 2);
    m_distortionVector.resize(W * H);
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            int index = y * W + x;
            m_distortionVector[index] = Point(res[index].x + offsetX, res[index].y + offsetY);
        }
    }

    initUndistortRectifyMap(parameters.camera.cameraMatrix, parameters.camera.distCoeff, Mat(), parameters.camera.undistCameraMatrix, parameters.camera.undistImageSize, CV_16SC2, m_map1, m_map2);
}

void
DistortionModel::undistortImage(const Mat& rawImg, Mat& res)
{
    remap(rawImg, res, m_map1, m_map2, INTER_LINEAR);
}

void
DistortionModel::undistortImage2(const Mat& rawImg, Mat& res)
{
    const int W = parameters.camera.imageSize.width;
    const int H = parameters.camera.imageSize.height;

    vector<Point> p(static_cast<unsigned long>(H * W)), resP;
    int ctmp = 0;
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            p[ctmp++] = Point(x, y);
        }
    }

    undistort(p, resP);

    const int siX = parameters.camera.undistImageSize.width;
    const int siY = parameters.camera.undistImageSize.height;
    res = Mat::zeros(Size(siX, siY), CV_8UC3);

    int counter = 0;

    const int rawChannels = rawImg.channels();
    const int rawSize = rawImg.rows * rawImg.cols;
    uchar* raw_D = rawImg.data;
    uchar* tmp_D = res.data;

    for (int i = 0; i < rawSize; i++) {
        int x = resP[counter].x;
        int y = resP[counter].y;
        counter++;
        raw_D += rawChannels;
        if (x < 0 || y < 0 || y >= siY || x >= siX) {
            continue;
        }
        uchar* currentP_tmp_D = tmp_D + (((y * siX) + x) * rawChannels);
        currentP_tmp_D[0] = raw_D[0];
        currentP_tmp_D[1] = raw_D[1];
        currentP_tmp_D[2] = raw_D[2];
    }
}

bool
DistortionModel::undistort(const vector<Point>& points, vector<Point2f>& res)
{
    res.resize(points.size());
    int W = parameters.camera.width;
    int H = parameters.camera.height;
    for (uint32_t i = 0; i < points.size(); ++i) {
        int x = points[i].x;
        int y = points[i].y;
        if (x < 0 || x >= W || y < 0 || y >= H) {
            printf("Error in undistort (%d, %d)\n", x, y);
            return false;
        }
        res[i] = m_distortionVector[y * W + x];
    }
    return true;
}

bool
DistortionModel::undistort(const vector<Point>& points, vector<Point>& res)
{
    res.resize(points.size());
    int W = parameters.camera.width;
    int H = parameters.camera.height;
    for (uint32_t i = 0; i < points.size(); ++i) {
        int x = points[i].x;
        int y = points[i].y;
        if (x < 0 || x >= W || y < 0 || y >= H) {
            printf("Error in undistort (%d, %d)\n", x, y);
            return false;
        }
        res[i] = m_distortionVector[y * W + x];
    }
    return true;
}

cv::Point2f DistortionModel::undistort(int x, int y) {
    int W = parameters.camera.width;
    int H = parameters.camera.height;
    if(x < 0 || x >= W || y < 0 || y >= H) {
        ROS_ERROR("error in undistort: (%d, %d)", x, y);
        return Point2f(-9999, -9999);
    }
    return m_distortionVector[y * W + x];
}

bool
DistortionModel::undistort_slow(const vector<Point>& points, vector<Point>& resPoints)
{
    Mat src = Mat(1, points.size(), CV_32FC2);
    Mat res = src;

    for (uint32_t i = 0; i < points.size(); ++i) {
        src.at<Vec2f>(0, i)[0] = points[i].x;
        src.at<Vec2f>(0, i)[1] = points[i].y;
    }

    undistortPoints(src, res, parameters.camera.cameraMatrix, parameters.camera.distCoeff, noArray(), parameters.camera.cameraMatrix);

    resPoints.resize(points.size());
    for (uint32_t i = 0; i < points.size(); ++i) {
        int x = res.at<Vec2f>(0, i)[0];
        int y = res.at<Vec2f>(0, i)[1];
        resPoints[i] = Point(x, y);
    }
    return true;
}

} // namespace dvision
