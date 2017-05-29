// Created on: May 20, 2017
//     Author: Wenxing Mei <mwx36mwx@gmail.com>

#include "dvision/camera.hpp"
#include <opencv2/opencv.hpp>

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "capture");
    ros::NodeHandle nh;

    dvision::Camera c;

    std::cout << "press c to capture" << std::endl;
    while (ros::ok()) {
        auto frame = c.capture();
        frame.show();
        char key = (char)cv::waitKey(1);
        if (key == 'c')
            frame.save("./");
    }
}
