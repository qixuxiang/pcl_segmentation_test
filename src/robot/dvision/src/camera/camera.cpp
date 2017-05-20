#include "dvision/camera.hpp"
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cassert>
#include <cerrno>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <sys/stat.h>
#include <fcntl.h>

#define CLEAR(x) memset(&(x), 0, sizeof((x)))

namespace dvision {
Camera::Camera(std::string device) : m_device(device) {
  init();
}

Camera::Camera(CameraSettings c) : m_device(c.device) {
  init();
  setCameraControl();
}

Camera::~Camera() {
  deInit();
}

void Camera::init() {
  try {
    openDevice();
    sleep(1);
    initFmt();
    setFrameRate(1, 30);
    initMmap();
    startIO();
    doIO();
  } catch (std::exception& e) {
    ROS_ERROR("Init camera error: %s", e.what());
    sleep(1);
    init();
  }
}

void Camera::deInit() {
  stopIO();
  closeDevice();
}

void Camera::initFmt() {
  struct v4l2_capability cap;
  CLEAR(cap);

  if (-1 == ioctl(m_fd, VIDIOC_QUERYCAP, &cap))
    throw std::runtime_error("Not a v4l2 device.");
  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    throw std::runtime_error("Not a video capture device");

  v4l2_format fmt;
  CLEAR(fmt);
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;
  fmt.fmt.pix.width = m_setting.width;
  fmt.fmt.pix.height = m_setting.height;
  __u32 pixfmt = V4L2_PIX_FMT_YUYV;

  fmt.fmt.pix.pixelformat = pixfmt;
  if (0 != ioctl(m_fd, VIDIOC_S_FMT, &fmt))
    throw std::runtime_error("Set format error");

  if (0 != ioctl(m_fd, VIDIOC_G_FMT, &fmt))
    throw std::runtime_error("Get format error");

  if (fmt.fmt.pix.width != m_setting.width ||
      fmt.fmt.pix.height != m_setting.height ||
      fmt.fmt.pix.pixelformat != pixfmt)
    throw std::runtime_error("Set format error");

  ROS_INFO("Set fmt success.");
}

void Camera::setFrameRate(uint32_t numerator, uint32_t denominator) {
  v4l2_streamparm parm;
  CLEAR(parm);

  parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (0 != ioctl(m_fd, VIDIOC_G_PARM, &parm))
    throw std::runtime_error("Get fps error");

  parm.parm.capture.timeperframe.numerator = numerator;
  parm.parm.capture.timeperframe.denominator = denominator;
  parm.parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
  if (0 != ioctl(m_fd, VIDIOC_S_PARM, &parm))
    throw std::runtime_error("Set fps error");

  // confirm
  if (0 != ioctl(m_fd, VIDIOC_G_PARM, &parm))
    throw std::runtime_error("Get fps error");
  if (parm.parm.capture.timeperframe.numerator != numerator ||
      parm.parm.capture.timeperframe.denominator != denominator)
    throw std::runtime_error("Set fps failed!");
  ROS_INFO("Set fps to %d/%d", numerator, denominator);
}

void Camera::initMmap() {
  struct v4l2_requestbuffers req;
  CLEAR(req);

  req.count = NUM_FRAME_BUFFERS;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (-1 == ioctl(m_fd, VIDIOC_REQBUFS, &req)) {
    if (errno == EINVAL)
      ROS_ERROR("Memory mapping not supported on this device!");
    else
      throw std::runtime_error("REQBUFS");
  }

  if (req.count < 1)
    throw std::runtime_error("Insufficient buffer memory.");

  CLEAR(buffers);

  for (n_buffers = 0; n_buffers < req.count; n_buffers++) {
    v4l2_buffer buf;
    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = n_buffers;

    if (-1 == ioctl(m_fd, VIDIOC_QUERYBUF, &buf))
      throw std::runtime_error("QUERYBUF");
    buffers[n_buffers].length = buf.length;
    buffers[n_buffers].start = reinterpret_cast<uint8_t *>(
        mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, m_fd,
             buf.m.offset));
    if (MAP_FAILED == buffers[n_buffers].start)
      throw std::runtime_error("MMAP");
  }

  // initial release all buffer space
  lastDequeued.index = UINT_MAX;
  for (unsigned int i = 0; i < n_buffers; i++) {
    struct v4l2_buffer buf;
    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    buf.timecode.type = 3;
    if (-1 == ioctl(m_fd, VIDIOC_QBUF, &buf))
      throw std::runtime_error("QBUF");
  }
  ROS_INFO("Init MMAP success.");
}

void Camera::startIO() {
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == ioctl(m_fd, VIDIOC_STREAMON, &type))
    throw std::runtime_error("Failed start capturing.");
}

void Camera::stopIO() {
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == ioctl(m_fd, VIDIOC_STREAMOFF, &type))
    throw std::runtime_error("Failed stop capturing.");
}

void Camera::set_control(V4L2CID id, const uint32_t value) {
  set_control(static_cast<uint32_t>(id), value);
}

void Camera::set_control(const uint32_t controlId,
                         const uint32_t controlValue) {
  v4l2_queryctrl queryctrl;
  CLEAR(queryctrl);
  queryctrl.id = controlId;
  if (ioctl(m_fd, VIDIOC_QUERYCTRL, &queryctrl) == -1) {
    if (errno != EINVAL)
      ROS_ERROR("query control error");
    else
      ROS_ERROR("%d not supported", controlId);
  } else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
    ROS_WARN("%s not supported", queryctrl.name);
  } else {
    v4l2_control control, current;
    CLEAR(control), CLEAR(current);
    control.id = controlId;
    control.value = controlValue;
    current.id = controlId;

    if (ioctl(m_fd, VIDIOC_S_CTRL, &control) == -1) {
      ROS_WARN("Set %s error", queryctrl.name);
    } else {
      ioctl(m_fd, VIDIOC_G_CTRL, &current);
      ROS_INFO("Set %s to %d", queryctrl.name, current.value);
    }
  }
}

void Camera::setCameraControl() {
  set_control(V4L2CID::white_balance_auto, m_setting.whitebalance_auto);
  set_control(V4L2CID::white_balance_temperature, m_setting.whitebalance_absolute);
  set_control(V4L2CID::exposure_auto, m_setting.exposure_auto);
  set_control(V4L2CID::exposure_absolute, m_setting.exposure_absolute);
  set_control(V4L2CID::focus_auto, m_setting.focus_auto);
  set_control(V4L2CID::focus_absolute, m_setting.focus_absolute);
  set_control(V4L2CID::brightness, m_setting.brightness);
  set_control(V4L2CID::contrast, m_setting.contrast);
  set_control(V4L2CID::saturation, m_setting.saturation);
  set_control(V4L2CID::sharpness, m_setting.sharpness);
  set_control(V4L2CID::gain, m_setting.gain);
  set_control(V4L2CID::hue, m_setting.hue);
  set_control(V4L2CID::gamma, m_setting.gamma);
}

void Camera::doIO() {
  try {
    auto beginTime = ros::Time::now();
    struct pollfd pollfd = {m_fd, POLLIN | POLLPRI, 0};
    int polled = poll( &pollfd, 1, 1000.0 / m_setting.frameRate * 6);  // wait for 6 frame for a new frame

    if (polled < 0)
      throw std::runtime_error("Cannot poll");
    else if (polled == 0)
      throw std::runtime_error("Poll timeout");
    else if (pollfd.revents & (POLLERR | POLLNVAL))
      throw std::runtime_error("Polling failed");

    auto stop1 = ros::Time::now();
    // release last buffer
    if (lastDequeued.index != UINT_MAX)
      if (-1 == ioctl(m_fd, VIDIOC_QBUF, &lastDequeued))
        throw std::runtime_error("Release buffer error");

    // dequeue raw camera buffer, we got one new raw frame, yuv
    CLEAR(lastDequeued);
    lastDequeued.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    lastDequeued.memory = V4L2_MEMORY_MMAP;

    if (-1 == ioctl(m_fd, VIDIOC_DQBUF, &lastDequeued))
      throw std::runtime_error("DQBUFF");
    auto stop2 = ros::Time::now();
    assert(lastDequeued.index < n_buffers);

    raw_yuv = buffers[lastDequeued.index].start;
    raw_yuv_size = lastDequeued.bytesused;

//    auto ioTime = stop1 - beginTime;
//    auto dequeTime = stop2 - stop1;
//    ROS_INFO("I/O: %lf ms, dequeue: %lf ms, size: %u", ioTime.toSec() * 1000, dequeTime.toSec() * 1000, raw_yuv_size);

    auto t = (stop2 - beginTime).toSec() * 1000;
    if(t > 40) {
      ROS_WARN("Camera I/O is getting slow, %lf.", t);
    }

  } catch (std::exception &e) {
    ROS_ERROR("Camera I/O failed, error: %s, trying to restart.", e.what());
    sleep(1);
//    stopIO();
    closeDevice();
    init();
  }
};

void Camera::openDevice() {
  struct stat buf;
  if (-1 == stat(m_device.c_str(), &buf)) {
    ROS_ERROR("Failed to stat %s", m_device.c_str());
    std::string devname = "/dev/video";
    int i = 0;
    while (true) {
      // try to open video0 up to video50
      int rc = 0;
      std::string tmp = devname + std::to_string(i);
      rc = stat(tmp.c_str(), &buf);
      if (rc == -1) {
        usleep(10000);
      } else {
        ROS_INFO("Found camera device: %s", tmp.c_str());
        m_device = tmp;
        break;
      }
      if (++i > 50) {
        ROS_WARN("No camera device available!");
        i = 0;
      }
    }
  }

  if (0 == S_ISCHR(buf.st_mode)) {
    throw std::runtime_error("Not a v4l2 device.");
  }

  m_fd = open(m_device.c_str(), O_CLOEXEC | O_RDWR);
  if (m_fd < 0) {
    throw std::runtime_error("Failed to open camera");
  } else {
    ROS_INFO("Opened camera %s", m_device.c_str());
  }
}

void Camera::closeDevice() {
  if (m_fd > 0) {
    close(m_fd);
  }
  m_fd = -1;
}

Frame Camera::capture() {
  doIO();
  return Frame(static_cast<const uint8_t*>(raw_yuv));
}

}