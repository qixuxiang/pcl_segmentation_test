#include "dvision/distortionModel.hpp"

using namespace std;
using namespace cv;

namespace dvision {
DistortionModel::DistortionModel()
{
}

DistortionModel::DistortionModel(Size imageSize, Mat cameraMatrix, Mat distCoeff)
{
    init(imageSize, cameraMatrix, distCoeff);
}

void
DistortionModel::init(Size imageSize, Mat cameraMatrix, Mat distCoeff)
{
    m_imageSize = imageSize;
    m_cameraMatrix = cameraMatrix;
    m_distCoeff = distCoeff;

    int W = m_imageSize.width;
    int H = m_imageSize.height;

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

    //    const int limit = 300;
    for (uint32_t i = 0; i < res.size(); ++i) {
        //        if(res[i].x > limit || res[i].x < -limit || res[i].y > limit || res[i].y < -limit) continue;

        int tmpW = max(maxW, abs(res[i].x - resCenter[0].x));
        int tmpH = max(maxH, abs(res[i].y - resCenter[0].y));
        if (tmpW > 700 || tmpH > 700)
            continue;
        maxW = tmpW;
        maxH = tmpH;
    }

    m_undistImageSize = Size(maxW * 2 + 1, maxH * 2 + 1);
    cout << "Undistorted image size:" << m_undistImageSize << endl;

    int offsetX = maxW - resCenter[0].x;
    int offsetY = maxH - resCenter[0].y;

    // (0, 0) (0, 1) (0, 2)
    // (1, 0) (1, 1) (1, 2)
    // (2, 0) (2, 1) (2, 2)

    m_newCameraMatrix = m_cameraMatrix;
    m_newCameraMatrix.at<double>(0, 2) += offsetX;
    m_newCameraMatrix.at<double>(1, 2) += offsetY;

    m_distortionVector.resize(W * H);
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            int index = y * W + x;
            m_distortionVector[index] = Point(res[index].x + offsetX, res[index].y + offsetY);
        }
    }

    initUndistortRectifyMap(m_cameraMatrix, m_distCoeff, Mat(), getOptimalNewCameraMatrix(m_cameraMatrix, m_distCoeff, m_imageSize, 1, m_imageSize, 0, true), m_imageSize, CV_16SC2, m_map1, m_map2);
}

void
DistortionModel::undistortImage(const Mat& rawImg, Mat& res)
{
    remap(rawImg, res, m_map1, m_map2, INTER_LINEAR);

    //  namedWindow("undist", CV_WINDOW_NORMAL);
    //  imshow("undist", res);

    //    const int W = m_imageSize.width;
    //    const int H = m_imageSize.height;
    //
    //    vector<Point> p(static_cast<unsigned long>(H * W)), resP;
    //    int ctmp = 0;
    //    for (int y = 0; y < H; y++) {
    //        for (int x = 0; x < W; x++) {
    //            p[ctmp++] = Point(x, y);
    //        }
    //    }
    //
    //    undistort(p, resP);
    //
    //    const int siX = m_undistImageSize.width;
    //    const int siY = m_undistImageSize.height;
    //    res = Mat::zeros(Size(siX, siY), CV_8UC3);
    //
    //    int counter = 0;
    //
    //    const int rawChannels = rawImg.channels();
    //    const int rawSize = rawImg.rows * rawImg.cols;
    //    uchar* raw_D = rawImg.data;
    //    uchar* tmp_D = res.data;
    //
    //    for (int i = 0; i < rawSize; i++) {
    //        int x = resP[counter].x;
    //        int y = resP[counter].y;
    //        counter++;
    //        raw_D += rawChannels;
    //        if (x < 0 || y < 0 || y >= siY || x >= siX) {
    ////            printf("Error In Programming %d %d \n", x, y);
    //            continue;
    //        }
    //        uchar* currentP_tmp_D = tmp_D + (((y * siX) + x) * rawChannels);
    //        currentP_tmp_D[0] = raw_D[0];
    //        currentP_tmp_D[1] = raw_D[1];
    //        currentP_tmp_D[2] = raw_D[2];
    //    }
}

void
DistortionModel::undistortImage2(const Mat& rawImg, Mat& res)
{
    const int W = m_imageSize.width;
    const int H = m_imageSize.height;

    vector<Point> p(static_cast<unsigned long>(H * W)), resP;
    int ctmp = 0;
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            p[ctmp++] = Point(x, y);
        }
    }

    undistort(p, resP);

    const int siX = m_undistImageSize.width;
    const int siY = m_undistImageSize.height;
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
    int W = m_imageSize.width;
    int H = m_imageSize.height;
    for (uint32_t i = 0; i < points.size(); ++i) {
        int x = points[i].x;
        int y = points[i].y;
        if(x < 0 || x >= W || y < 0 || y >= H) {
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
    int W = m_imageSize.width;
    int H = m_imageSize.height;
    for (uint32_t i = 0; i < points.size(); ++i) {
        int x = points[i].x;
        int y = points[i].y;
        if(x < 0 || x >= W || y < 0 || y >= H) {
            printf("Error in undistort (%d, %d)\n", x, y);
            return false;
        }
        res[i] = m_distortionVector[y * W + x];
    }
    return true;
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

    undistortPoints(src, res, m_cameraMatrix, m_distCoeff, noArray(), m_cameraMatrix);

    resPoints.resize(points.size());
    for (uint32_t i = 0; i < points.size(); ++i) {
        int x = res.at<Vec2f>(0, i)[0];
        int y = res.at<Vec2f>(0, i)[1];
        resPoints[i] = Point(x, y);
    }
    return true;
}

} // namespace dvision
