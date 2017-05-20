#pragma once
#include <linux/videodev2.h>
#include <string>
#include <cstdint>
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

  std::string device;
  std::uint32_t width, height;
  std::uint32_t frameRate;
  std::uint32_t brightness;
  std::uint32_t contrast;
  std::uint32_t saturation;
  std::uint32_t hue;
  std::uint32_t sharpness;
  std::uint32_t gain;
  std::uint32_t gamma;

  std::uint32_t whitebalance_auto;
  std::uint32_t whitebalance_absolute;
  std::uint32_t exposure_auto;
  std::uint32_t exposure_absolute;
  std::uint32_t focus_auto;
  std::uint32_t focus_absolute;
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