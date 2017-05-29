#include "dvision/camera.hpp"
#include "dvision/distortionModel.hpp"
#include <gtest/gtest.h>

using namespace dvision;
using namespace cv;
using namespace std;

TEST(distortionModel, main)
{
    Mat cameraMatrix = (Mat_<double>(3, 3) << 360.591090231311, 0, 310.7131585594641, 0, 360.4918824799427, 252.0890520277582, 0, 0, 1);

    Mat distCoeff = (Mat_<double>(1, 14) << 12.17446992931163,
                     9.674181625244897,
                     -9.979700211544175e-05,
                     -0.0001482121838188539,
                     0.6561453924261769,
                     12.50596489391226,
                     13.94603005487614,
                     2.886257323576563,
                     0.001526969658511264,
                     -0.0002591965834689962,
                     0.0001776014517591451,
                     7.26029339820618e-05,
                     0.0005823374570643553,
                     0.001257762144819584);

    DistortionModel dist(Size(640, 480), cameraMatrix, distCoeff);

    dvision::Camera c;
    while (ros::ok()) {
        auto frame = c.capture();
        Mat dst;
        dist.undistortImage(frame.getRGB(), dst);
        namedWindow("undist", CV_WINDOW_NORMAL);
        imshow("undist", dst);
        waitKey(1);
    }
}

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "capture");
    ros::NodeHandle nh;

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}