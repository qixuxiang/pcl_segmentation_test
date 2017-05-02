// Created on: May 2, 2017
//     Author: Wenxing Mei <mwx36mwx@gmail.com>
// Wrapper for std::thread

#include <pthread.h>
#include <thread>
#include <ros/ros.h>

namespace dprocess {

template<typename T>
class DProcess {
public:
  DProcess() : m_rt(false) {}
  explicit DProcess(bool rt) : m_rt(rt) {}
  void spin() {
    m_thread = std::thread([=] {
      // maybe add some timer
      // todo add watch dog
      while (true) {
        static_cast<T *>(this)->tick();
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
  void spinOnce() {
    m_thread = std::thread([=] {
      static_cast<T *>(this)->tick();
    });
    set_policy();
  }

  void join() {
    m_thread.join();
  }

private:
  virtual void tick() {}
  bool m_rt;
  std::thread m_thread;
};

}