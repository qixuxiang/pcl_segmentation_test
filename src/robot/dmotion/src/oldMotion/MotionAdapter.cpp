#include <Utilities/FileLogger.hpp>
#include "MotionAdapter.hpp"
#include "Blackboard/Blackboard.hpp"
#include "GaitStateManager.hpp"
#include "Utilities/Timer.hpp"
#include "Utilities/Console.h"
#include "Thread/ThreadManager.hpp"
#ifdef __linux__
#include "Utilities/options.hpp"
#endif

using namespace std;
using namespace dancer2050;


MotionAdapter::MotionAdapter(Blackboard *bb) : Adapter(bb), timestamp(getCurrentTime()), fps(0) {
  readOptions(bb->config);
  RobotPara::readOptions(bb->config);
  manager = new GaitStateManager(bb);
}

/*-----------------------------------------------------------------------------
 * Motion thread tick function
 *---------------------------------------------------------------------------*/

void MotionAdapter::tick() {
  cycle++;
  request = readFrom(behaviour, actions);
  manager->checkNewCommand(request);
  manager->tick();

  /* Write info to blackboard.motion */
  // writeTo(motion, uptime, time(nullptr) - startTime);
  // writeTo(motion, timestamp, getCurrentTime());

//  auto current_time = getCurrentTime();
//  fps = 1000000 / (double)(current_time - timestamp);
//  timestamp = current_time;
//  auto stable = readFrom(motion, stable) == stabilityStatus::stable;
//  flog("./motion_fps.log") << fps << " " << stable << endl;
}


#endif
