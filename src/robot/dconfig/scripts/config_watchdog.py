#!/usr/bin/env python
"""
Created on: May 2, 2017
    Author: Wenxing Mei <mwx36mwx@gmail.com>

TOPIC: 'reload_config'
cwd: $HOME/humanoid/src/dconfig/scripts
Watching directory: ../dmotion
Launched by dmotion/launch/dmotion.launch
Watch config directory, call `rosparam load` and publish msg when file is changed.
"""

import subprocess
import rospy
import os
from std_msgs.msg import String
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

PATH = '../'
MOTION_CONFIG = [
    'motion.yml',
    'motor.yml',
    'kick.yml',
    'setup.yml',
    'pvhipY.yml'
]
MOTION_TOPIC = '/humanoid/ReloadMotionConfig'
VISION_CONFIG = [
    'localization.yml'
]
VISION_TOPIC = '/humanoid/ReloadVisionConfig'


class Handler(FileSystemEventHandler):
    """File Handler."""

    def __init__(self):
        """Init."""
        super(Handler, self).__init__()
        self.pub_motion = rospy.Publisher(MOTION_TOPIC, String, queue_size=1)
        self.pub_vision = rospy.Publisher(VISION_TOPIC, String, queue_size=1)
        rospy.init_node('config_watchdog')
        rospy.loginfo("CWD: %s" % os.getcwd())

    def on_modified(self, event):
        """Callback on modified."""
        if not event.is_directory:
            _, filename = os.path.split(event.src_path)
            if filename.split('.')[-1] in ['yml', 'yaml']:
                for cfg in MOTION_CONFIG:
                    self.pub_motion.publish('')
                    subprocess.call(
                        ['rosparam', 'load', '%s/dmotion/%s' % (PATH, cfg)])
                for cfg in VISION_CONFIG:
                    self.pub_vision.publish('')
                    subprocess.call(
                        ['rosparam', 'load', '%s/dvision/%s' % (PATH, cfg)])
                    # print 'rosparam load %s/%s' % (PATH, cfg)
                # debug
                rospy.loginfo('config changed, published reload msg')
            else:
                rospy.logwarn('%s not yaml file' % event.src_path)


if __name__ == '__main__':
    try:
        event_handler = Handler()
        observer = Observer()
        observer.schedule(event_handler, path=PATH, recursive=True)
        observer.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
