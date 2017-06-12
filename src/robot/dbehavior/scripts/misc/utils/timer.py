"""
Timer.

A timing utility that can measure time and meet time targets with ease.
rospy.Time is used to measure time, so the time units are nanoseconds.
The restart(), start(), and stop() methods are chainable, e.g.:
countdown_timer = Timer(1000000).start()
countdown_timer.restart().start()
"""
# @Author: Yusu Pan <yuthon>
# @Date:   2017-06-12T22:28:14+08:00
# @Email:  xxdsox@gmail.com
# @Project: humanoid
# @Filename: timer.py
# @Last modified by:   yuthon
# @Last modified time: 2017-06-12T22:28:31+08:00
# @Copyright: ZJUDancer

import rospy


def current_milli_time():
    """Get current time in millisecond."""
    return int(round(rospy.get_rostime().to_sec() * 1000))


def get_current_time():
    """Get current time in nanosecond."""
    return rospy.get_rostime().to_nsec()


class Timer(object):
    """Timer."""

    def __init__(self, time_target=0):
        """Init."""
        super(Timer, self).__init__()
        self.timeTarget = rospy.Duration.from_sec(time_target)
        self.running = False
        self.elapsedTime = rospy.Duration.from_sec(0)
        # self.startTime = blackboard.behaviour.timestamp
        self.startTime = rospy.get_rostime()
        self.restart()

    def restart(self):
        """Reset the timer.

        If it's running at the time, it will keep running.
        """
        self.elapsedTime = rospy.Duration.from_sec(0)
        self.startTime = rospy.get_rostime()
        return self

    def start(self):
        """Start the timer.

        Does nothing if it's already running.
        """
        if not self.running:
            self.startTime = rospy.get_rostime()
            self.running = True
        return self

    def stop(self):
        """Stop the timer.

        Does nothing if it's already stopped.
        """
        if self.running:
            self.elapsedTime += rospy.get_rostime() - self.startTime
            self.running = False
        return self

    def elapsed(self):
        """Return how much time has elapsed so far in milliseconds."""
        elapsedTime = self.elapsedTime + rospy.get_rostime() - self.startTime
        return int(round(elapsedTime.to_sec() * 1000000))

    def elapsed_sec(self):
        """Return how much time has elapsed so far in seconds."""
        elapsedTime = self.elapsedTime + rospy.get_rostime() - self.startTime
        return int(round(elapsedTime.to_sec()))

    def leftshift(self, left_time):
        """Leftshift time (in seconds)."""
        self.startTime -= rospy.Duration.from_sec(left_time)

    def finished(self):
        """Return wheather we have reached our target time of not."""
        return self.elapsed() >= self.timeTarget
