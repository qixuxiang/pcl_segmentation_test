// Created on: May 2, 2017
//     Author: Wenxing Mei <mwx36mwx@gmail.com>
// Wrapper for std::thread
#pragma once
#include <pthread.h>
#include <thread>
#include <ros/ros.h>

namespace dprocess {

template<typename T>
class DProcess {
public:
  explicit DProcess(int freq, bool rt = false) : m_freq(freq), m_rt(rt) {}
  virtual ~DProcess() {}
  void spin() {
    m_thread = std::thread([=] {
      // maybe add some timer
      // todo add watch dog
      ros::Rate r(m_freq);
      while (ros::ok()) {
        static_cast<T *>(this)->tick();
        ros::spinOnce();
        r.sleep();
      }
    });

    set_policy();
  }

  void set_policy() {
    if(m_rt) {
      sched_param param;
      param.sched_priority = 99;
      // todo, needs sudo
      if(pthread_setschedparam(m_thread.native_handle(), SCHED_FIFO, &param)) {
        ROS_INFO("Set REAL TIME policy success.");
      } else {
        ROS_WARN("I can't run in REAL TIME.");
      }
    }
  }

  // only for test
  // TODO(MWX): need boosting
  void spinOnce() {
    m_thread = std::thread([=] {
      static_cast<T *>(this)->tick();
    });
  }

  void join() {
    m_thread.join();
  }

  virtual void tick() {}

private:
  int m_freq;
  bool m_rt;

  std::thread m_thread;
};

}