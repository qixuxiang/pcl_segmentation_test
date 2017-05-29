#include "dvision/distortionModel.hpp"

using namespace std;
using namespace cv;

namespace dvision {
DistortionModel::DistortionModel(Size imageSize, Mat cameraMatrix, Mat distCoeff)
  : m_imageSize(imageSize)
  , m_cameraMatrix(cameraMatrix)
  , m_distCoeff(distCoeff)
{
    init();
}

void
DistortionModel::init()
{
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
        if(tmpW > 700 || tmpH > 700) continue;
        maxW = tmpW;
        maxH = tmpH;
    }

    m_undistImageSize = Size(maxW * 2 + 1, maxH * 2 + 1);
    cout << "Undistorted image size:" << m_undistImageSize << endl;

    int offsetX = maxW - resCenter[0].x;
    int offsetY = maxH - resCenter[0].y;

//    int offsetX = (m_undistImageSize.width - W) / 2.;
//    int offsetY = (m_undistImageSize.height - H) / 2.;

    m_distortionVector.resize(W * H);
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            int index = y * W + x;
            m_distortionVector[index] = Point(res[index].x + offsetX, res[index].y + offsetY);

            if (x < 0 || y < 0)
                cout << Point(x, y) << endl;
        }
    }
}

void
DistortionModel::undistortImage(const Mat& rawImg, Mat& res)
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
//            printf("Error In Programming %d %d \n", x, y);
            continue;
        }
        uchar* currentP_tmp_D = tmp_D + (((y * siX) + x) * rawChannels);
        currentP_tmp_D[0] = raw_D[0];
        currentP_tmp_D[1] = raw_D[1];
        currentP_tmp_D[2] = raw_D[2];
    }
}

void
DistortionModel::undistort(const vector<Point>& points, vector<Point>& res)
{
    res.resize(points.size());
    for (uint32_t i = 0; i < points.size(); ++i) {
        int x = points[i].x;
        int y = points[i].y;

        res[i] = m_distortionVector[y * m_imageSize.width + x];
    }
}

void
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
}

} // namespace dvision
