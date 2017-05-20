#pragma once
#include <linux/videodev2.h>
#include <string>
#include <cstdint>
#include <ros/ros.h>

namespace dvision {
struct CameraSettings {
  inline explicit CameraSettings()
      : device("/dev/video0"),
        width(640),
        height(480),
        frameRate(30),
        brightness(128),
        contrast(128),
        saturation(128),
        hue(0),
        sharpness(128),
        gain(255),
        gamma(32533),
        whitebalance_auto(0),
        whitebalance_absolute(4500),
        exposure_auto(1),
        exposure_absolute(150),
        focus_auto(1),
        focus_absolute(0) {}

  inline explicit CameraSettings(ros::NodeHandle *nh) {
    if (!nh->getParam("/dvision/camera/device", device)) { ROS_FATAL("CameraSettings get pararm error"); }
    if (!nh->getParam("/dvision/camera/width", width)) { ROS_FATAL("CameraSettings get pararm error"); }
    if (!nh->getParam("/dvision/camera/height", height)) { ROS_FATAL("CameraSettings get pararm error"); }
    if (!nh->getParam("/dvision/camera/frameRate", frameRate)) { ROS_FATAL("CameraSettings get pararm error"); }
    if (!nh->getParam("/dvision/camera/brightness", brightness)) { ROS_FATAL("CameraSettings get pararm error"); }
    if (!nh->getParam("/dvision/camera/contrast", contrast)) { ROS_FATAL("CameraSettings get pararm error"); }
    if (!nh->getParam("/dvision/camera/saturation", saturation)) { ROS_FATAL("CameraSettings get pararm error"); }
    if (!nh->getParam("/dvision/camera/hue", hue)) { ROS_FATAL("CameraSettings get pararm error"); }
    if (!nh->getParam("/dvision/camera/sharpness", sharpness)) { ROS_FATAL("CameraSettings get pararm error"); }
    if (!nh->getParam("/dvision/camera/gain", gain)) { ROS_FATAL("CameraSettings get pararm error"); }
    if (!nh->getParam("/dvision/camera/gamma", gamma)) { ROS_FATAL("CameraSettings get pararm error"); }
    if (!nh->getParam("/dvision/camera/whitebalance_auto", whitebalance_auto)) { ROS_FATAL("CameraSettings get pararm error"); }
    if (!nh->getParam("/dvision/camera/whitebalance_absolute", whitebalance_absolute)) { ROS_FATAL("CameraSettings get pararm error"); }
    if (!nh->getParam("/dvision/camera/exposure_auto", exposure_auto)) { ROS_FATAL("CameraSettings get pararm error"); }
    if (!nh->getParam("/dvision/camera/exposure_absolute", exposure_absolute)) { ROS_FATAL("CameraSettings get pararm error"); }
    if (!nh->getParam("/dvision/camera/focus_auto", focus_auto)) { ROS_FATAL("CameraSettings get pararm error"); }
    if (!nh->getParam("/dvision/camera/focus_absolute", focus_absolute)) { ROS_FATAL("CameraSettings get pararm error"); }
  }

  std::string device;
  int width, height;
  int frameRate;
  int brightness;
  int contrast;
  int saturation;
  int hue;
  int sharpness;
  int gain;
  int gamma;

  int whitebalance_auto;
  int whitebalance_absolute;
  int exposure_auto;
  int exposure_absolute;
  int focus_auto;
  int focus_absolute;
};

enum class V4L2CID {
  white_balance_auto = V4L2_CID_AUTO_WHITE_BALANCE,
  white_balance_temperature = V4L2_CID_WHITE_BALANCE_TEMPERATURE,
  exposure_auto = V4L2_CID_EXPOSURE_AUTO,
  exposure_absolute = V4L2_CID_EXPOSURE_ABSOLUTE,
  focus_auto = V4L2_CID_FOCUS_AUTO,
  focus_absolute = V4L2_CID_FOCUS_ABSOLUTE,
  brightness = V4L2_CID_BRIGHTNESS,
  contrast = V4L2_CID_CONTRAST,
  saturation = V4L2_CID_SATURATION,
  sharpness = V4L2_CID_SHARPNESS,
  gain = V4L2_CID_GAIN,
  hue = V4L2_CID_HUE,
  gamma = V4L2_CID_GAMMA
};
}