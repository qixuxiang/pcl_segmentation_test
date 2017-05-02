#pragma once
#include <dprocess/dprocess.hpp>
using dprocess::DProcess;
namespace dmotion {
class DMotion: public DProcess<DMotion> {
public:
  explicit DMotion(ros::NodeHandle* n);
  ~DMotion();
  void tick() override;

private:
  ros::NodeHandle* m_nh;
};
}