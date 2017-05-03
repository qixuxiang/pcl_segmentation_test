#pragma once
#include "misc/configclient/configclient.hpp"
#include "misc/types/streamable.hpp"
#include <unordered_map>

using namespace std;
using namespace dancer2050;

struct MotionConfig : public Streamable<MotionConfig> {
  // sections
  unordered_map<string, string> robot;
  unordered_map<string, string> motor;
  unordered_map<string, string> hardware;

  // update robotpara
  void update();

  MSGPACK_DEFINE_MAP(robot, motor, hardware)
};

typedef ConfigClient<MotionConfig> MotionConfigClient;
