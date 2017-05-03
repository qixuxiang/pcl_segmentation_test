#pragma once
#include <boost/program_options.hpp>
#include "Types/ActionCommand.hpp"
#include "Blackboard/Adapter.hpp"

#ifdef __linux__
#include <sys/inotify.h>
#endif

#define INBUF_LEN 32 * (sizeof(struct inotify_event) + 16)
/**
 * MotionAdapter - interfaces between Motion & rest of the system via Blackboard
 *
 * The MotionAdapter is the controlling class for evething Motion. It reads
 * ActionCommands from the Blackboard.
 *
 * It then passes AC::Head, AC::Body to a Generator instance,
 * which process them and determines the correct JointValues for the next DCM
 * cycle. In practice, the Generator will usually be a DistributorGenerator,
 * which will select the most appropriate Generator based on the AC's.
 *
 * These JointValues are passed to an Effector instance. The
 * effector actuated the joints according to thes instructions.
 */
class transitHub;
class GaitStateManager;
class MotionAdapter : public Adapter {
 public:
  /* Constructor */
  MotionAdapter(Blackboard* bb);
  /* Destructor */
  ~MotionAdapter();
  /* One cycle of this thread */
  void tick();
  /* Read values from the global options */


 private:
  GaitStateManager* manager;
  ActionCommand::All request;
  std::vector<ActionCommand::gaitV> gaitvs;
#ifdef __linux__
  // used for inotify
  void startInotify();
  bool inotify_Check();
  void reload_param();

  int inotify_fd;
  char inotify_buf[INBUF_LEN];
  fd_set inotify_fdss;
  struct timeval inotify_timeout;
  int wd;
#endif
  bool watch_config;
  int cycle;
  time_t startTime;
  // calculating fps
  int64_t timestamp;
  double fps;
};
