#include "dvision/distortionModel.hpp"
#include "dvision/camera.hpp"
#include <gtest/gtest.h>

using namespace dvision;
using namespace cv;
using namespace std;

TEST(distortionModel, main)
{
    Mat cameraMatrix = (Mat_<double>(3, 3) << 375.0329245163845, 0, 333.425467117812, 0, 371.7363174866422, 258.6764129612715, 0, 0, 1);

    Mat distCoeff = (Mat_<double>(1, 14) << -0.1238699435322457,
                     2.272671826783335,
                     0.001680597501468876,
                     -0.0004516286381935245,
                     0.348627835181028,
                     0.2261706238550091,
                     2.174596312337722,
                     1.127177366910493,
                     0.009636570316125281,
                     -0.001494059533024259,
                     -0.04040591328900087,
                     0.01032415996679334,
                     0.03141385670627844,
                     0.006697454979984255);


    DistortionModel dist(Size(640, 480), cameraMatrix, distCoeff);

    dvision::Camera c;
    while (ros::ok()) {
        auto frame = c.capture();
        frame.show();


        Mat dst;
//        imshow("raw", img);
//        waitKey(0);
        dist.undistortImage(frame.getRGB(), dst);

        namedWindow("undist", CV_WINDOW_NORMAL);
        imshow("undist", dst);
//        waitKey(0);
    }

//    Mat img = imread("/home/mwx/Pictures/calibration/1495341982875198303.png"), dst;
//    imshow("raw", img);
//    waitKey(0);
//    dist.undistortImage(img, dst);
//
//    namedWindow("undist", CV_WINDOW_NORMAL);
//    imshow("undist", dst);
//    waitKey(0);
}

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "capture");
    ros::NodeHandle nh;



    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

// -------------cameraMatrix--------------
// [3 x 3]
// [375.0329245163845, 0, 333.425467117812;
//  0, 371.7363174866422, 258.6764129612715;
//  0, 0, 1]
// ---------------distCoeffs--------------
// [1 x 14]
// [-0.1238699435322457;
//  2.272671826783335;
//  0.001680597501468876;
//  -0.0004516286381935245;
//  0.348627835181028;
//  0.2261706238550091;
//  2.174596312337722;
//  1.127177366910493;
//  0.009636570316125281;
//  -0.001494059533024259;
//  -0.04040591328900087;
//  0.01032415996679334;
//  0.03141385670627844;
//  0.006697454979984255]

/////////////////////////////////////////////////////////////////

//-------------cameraMatrix--------------
//[3 x 3]
//[541.7737837385166, 0, 336.3797324745365;
//0, 557.1712224431056, 260.3594414394327;
//0, 0, 1]
//---------------distCoeffs--------------
//[1 x 14]
//[-0.2356557598067911;
//-0.1973772071091949;
//0.1202405030054182;
//-0.05184992483277222;
//-4.618324301522162;
//0.2649550078359445;
//-0.1879089355733787;
//-6.239008853537703;
//0.0195117367647637;
//0.07327139588475912;
//-0.175018117037268;
//0.01692410643592426;
//-0.2009823760679314;
//-0.172131013674498]