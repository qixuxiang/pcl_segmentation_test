#pragma once
#include <opencv2/opencv.hpp>
#include "dvision/frame.hpp"
#include <dirent.h>
#include <string>
#include <vector>


namespace dvision{
class CameraDummy
{
public:
  CameraDummy(std::string imageFolder){
      init(imageFolder);
  }
  bool init(std::string imageFolder);
  cv::Mat capture();

private:
  std::vector<std::string> imagesName;
  int imagesNum;
  int imageCnt = 0;
  std::string path;

};
}
