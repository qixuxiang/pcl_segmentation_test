#pragma once
#include "dmotion/GaitType.h"
#include "dmotion/ActionCmd.h"

namespace dmotion {
class GaitStateBase;
class GaitStateWenxi;
class GaitStateCrouch;
class GaitStateStandup;
class GaitStateKick;
class GaitStateGoalie;
class GaitStateSetup;

class GaitStateManager {
public:
  GaitStateManager();
  ~GaitStateManager();
  void tick();
  void checkNewCommand(ActionCmd::ConstPtr& request);

};
}