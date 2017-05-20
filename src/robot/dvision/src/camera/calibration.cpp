#include <iostream>
#include <cstdio>
#include <opencv2/opencv.hpp>
#include <dirent.h>

using namespace std;
using namespace cv;
class CalibSettings {
public:
  enum Pattern {
    NOT_EXISTING,
    CHESSBOARD,
    CIRCLES_GRID,
    ASYMMETRIC_CIRCLES_GRID
  };
  CalibSettings()
      :
      calibMode(CHESSBOARD) {}
  void init(int boardWidth, int boardHeight, float squareSize) {
    this->boardWidth = boardWidth;
    this->boardHeight = boardHeight;
    this->squareSize = squareSize;
  }
  int getFlag() {
    int flag = 0;
//      		flag |= CV_CALIB_USE_INTRINSIC_GUESS; // keep camera matrix as
    //zero 		flag |= CV_CALIB_FIX_ASPECT_RATIO; 		flag |=
    //CV_CALIB_FIX_PRINCIPAL_POINT; 		flag |=
    //CV_CALIB_ZERO_TANGENT_DIST; 		flag |=
    //CV_CALIB_FIX_FOCAL_LENGTH; 		flag |= CV_CALIB_FIX_K1;
    //		flag |= CV_CALIB_FIX_K2;
//      flag |= CV_CALIB_FIX_K3;
//      if (!params.calib->high_dimension()) {
//        flag |= CV_CALIB_FIX_K4;
//        flag |= CV_CALIB_FIX_K5;
//        flag |= CV_CALIB_FIX_K6;
//      }
    flag |= CV_CALIB_RATIONAL_MODEL;
    flag |= CV_CALIB_THIN_PRISM_MODEL;
    return flag;
  }
  cv::Size getBoardSize() {
    return cv::Size(boardWidth, boardHeight);
  }
  float getSquareSize() {
    return squareSize;
  }
  Pattern getCalibMode() {
    return calibMode;
  };

private:
  int boardWidth = 10;
  int boardHeight = 10;
  float squareSize = 30;
  Pattern calibMode;
};

static double computeReprojectionErrors(const vector<vector<cv::Point3f>> &objectPoints,
                                        const vector<vector<cv::Point2f>> &imagePoints,
                                        const vector<cv::Mat> &rvecs, const vector<cv::Mat> &tvecs,
                                        const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
                                        vector<float> &perViewErrors) {
  vector<cv::Point2f> imagePoints2;
  int i, totalPoints = 0;
  double totalErr = 0, err;
  perViewErrors.resize(objectPoints.size());

  for (i = 0; i < (int) objectPoints.size(); ++i) {
    projectPoints(cv::Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
                  distCoeffs, imagePoints2);
    err = norm(cv::Mat(imagePoints[i]), cv::Mat(imagePoints2), cv::NORM_L2);

    int n = (int) objectPoints[i].size();
    perViewErrors[i] = (float) std::sqrt(err * err / n);
    totalErr += err * err;
    totalPoints += n;
  }

  return std::sqrt(totalErr / totalPoints);
}

static void calcBoardCornerPositions(cv::Size boardSize, float squareSize,
                                     vector<cv::Point3f> &corners,
                                     CalibSettings::Pattern patternType) {
  corners.clear();

  switch (patternType) {
    case CalibSettings::CHESSBOARD:
    case CalibSettings::CIRCLES_GRID:
      for (int i = 0; i < boardSize.height; ++i)
        for (int j = 0; j < boardSize.width; ++j)
          corners.push_back(
              cv::Point3f(j * squareSize, i * squareSize, 0));
      break;

    case CalibSettings::ASYMMETRIC_CIRCLES_GRID:
      for (int i = 0; i < boardSize.height; i++)
        for (int j = 0; j < boardSize.width; j++)
          corners.push_back(cv::Point3f((2 * j + i % 2) * squareSize,
                                        i * squareSize, 0));
      break;
    default:
      break;
  }
}

static bool runCalibration(CalibSettings &s, cv::Size &imageSize, cv::Mat &cameraMatrix,
                           cv::Mat &distCoeffs, vector<vector<cv::Point2f>> imagePoints,
                           vector<cv::Mat> &rvecs, vector<cv::Mat> &tvecs,
                           vector<float> &reprojErrs, double &totalAvgErr) {
  cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
  if (s.getFlag() & cv::CALIB_FIX_ASPECT_RATIO)
    cameraMatrix.at<double>(0, 0) = 1.0;

  distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

  vector<vector<cv::Point3f>> objectPoints(1);
  calcBoardCornerPositions(s.getBoardSize(), s.getSquareSize(), objectPoints[0], s.getCalibMode());

  objectPoints.resize(imagePoints.size(), objectPoints[0]);

  // Find intrinsic and extrinsic camera parameters
  double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
                               distCoeffs, rvecs, tvecs, s.getFlag());

  cout << "Re-projection error reported by calibrateCamera: " << rms << endl;

  bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

  totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs,
                                          cameraMatrix, distCoeffs, reprojErrs);

  return ok;
}

bool runCalibrationAndSave(CalibSettings &s, cv::Size imageSize, cv::Mat &cameraMatrix,
                           cv::Mat &distCoeffs,
                           vector<vector<cv::Point2f>> imagePoints) {
  vector<cv::Mat> rvecs, tvecs;
  vector<float> reprojErrs;
  double totalAvgErr = 0;

  bool ok = runCalibration(s, imageSize, cameraMatrix, distCoeffs, imagePoints,
                           rvecs, tvecs, reprojErrs, totalAvgErr);
  cout << (ok ? "Calibration succeeded" : "Calibration failed")
       << ". avg re projection error = " << totalAvgErr;

  return ok;
}
int main() {
  CalibSettings s;                             //Calibration settings
  vector<vector<cv::Point2f>> imagePoints;
  cv::Mat cameraMatrix, distCoeffs;
  cv::Size imageSize;

  int imagesNum = 0;
  int foundCornerNum = 0;

  vector<string> imagesName;
  //string pathDirectory = "/home/yyj/ZJUDancer/imageProcess/cv_test2/build/";
  //TODO edit calibrating image's path
  string pathDirectory = "/home/yyj/xx/calibrationimage/";
  DIR *dir;
  struct dirent *ent;
  if ((dir = opendir(pathDirectory.c_str())) != NULL) {
    /* print all the files and directories within directory */
    while ((ent = readdir(dir)) != NULL) {
      string tmpFileName = ent->d_name;
      if (tmpFileName.length() > 4) {
        auto nPos = tmpFileName.find(".jpg");
        if (nPos != string::npos) {
          imagesName.push_back(tmpFileName);
          imagesNum++;
        }
      }
    }
    closedir(dir);
  } else {
    perror("");
    return EXIT_FAILURE;
  }

  sort(imagesName.begin(), imagesName.end());

  for (auto image_name : imagesName) {
    cv::Mat view;
    view = cv::imread((pathDirectory + image_name).c_str());
    imageSize = view.size();
    vector<cv::Point2f> pointBuf;
    bool found;
    found = findChessboardCorners(view, s.getBoardSize(), pointBuf,
                                  cv::CALIB_CB_ADAPTIVE_THRESH |
                                      cv::CALIB_CB_FAST_CHECK |
                                      cv::CALIB_CB_NORMALIZE_IMAGE);
    if (found) {
      cv::Mat viewGray;
      cvtColor(view, viewGray, cv::COLOR_BGR2GRAY);
      cornerSubPix(
          viewGray, pointBuf, cv::Size(11, 11), cv::Size(-1, -1),
          cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));

      imagePoints.push_back(pointBuf);
      drawChessboardCorners(view, s.getBoardSize(), cv::Mat(pointBuf), found);
      foundCornerNum++;
      cout << image_name << " found corner successfully!" << endl;
    } else {
      cout << image_name << " found corner failed! & removed!" << endl;
      remove((pathDirectory + image_name).c_str());
    }
  }

  cout << foundCornerNum << " of " << imagesNum << " found corner successfully!" << endl;
  runCalibrationAndSave(s, imageSize, cameraMatrix, distCoeffs,
                        imagePoints);
  cout << "-------------cameraMatrix--------------" << endl;

  cout << cameraMatrix.size() << endl;
  cout << cameraMatrix << endl;
  cout << "---------------distCoeffs--------------" << endl;
  cout << distCoeffs.size() << endl;
  cout << distCoeffs << endl;

  for (auto image_name : imagesName) {
    cv::Mat view = cv::imread((pathDirectory + image_name).c_str());
    cv::Mat temp = view.clone();
    undistort(temp, view, cameraMatrix, distCoeffs);
    cv::imshow("undistorted", view);
    cv::waitKey(0);
  }

  return 0;
}