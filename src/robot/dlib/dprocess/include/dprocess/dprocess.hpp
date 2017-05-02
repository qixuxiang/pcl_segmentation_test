// Created on: May 2, 2017
//     Author: Wenxing Mei <mwx36mwx@gmail.com>
// Wrapper for std::thread

#include <pthread.h>
#include <thread>

namespace dprocess {

static const int RT_PRIORITY = 1;
static const int RT_POLICY = SCHED_FIFO;

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
  }

  void spinOnce() {
    m_thread = std::thread([=] {
      static_cast<T *>(this)->tick();
    });
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