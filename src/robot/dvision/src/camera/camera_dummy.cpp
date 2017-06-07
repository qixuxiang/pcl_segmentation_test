#include <dvision/camera_dummy.hpp>
namespace dvision{

bool CameraDummy::init(std::string imageFolder){
  path = imageFolder;
  DIR *dir;
  struct dirent *ent;
  if ((dir = opendir (imageFolder.c_str())) != NULL) {
      /* print all the files and directories within directory */
      while ((ent = readdir (dir)) != NULL) {
          std::string tmpFileName = ent->d_name;
          if(tmpFileName.length() > 4){
              auto nPos = tmpFileName.find(".jpg");
              if(nPos != std::string::npos){
                  imagesName.push_back(tmpFileName);
                  imagesNum++;
              }
          }

//      printf ("%s\n", ent->d_name);
      }
      closedir (dir);
  } else {
      /* could not open directory */
      perror ("");
      return EXIT_FAILURE;
  }
}

cv::Mat
CameraDummy::capture(){
  cv::Mat res = cv::imread((path + imagesName[imageCnt]).c_str());
  if(imageCnt < imagesNum){
    imageCnt++;
  }
  return res;
  // return Frame(static_cast<uint8_t*>(res.data), 1920, 1080);
}


}
