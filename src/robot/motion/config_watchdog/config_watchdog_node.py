#!/usr/bin/env python
"""
Created on: May 2, 2017
    Author: Wenxing Mei <mwx36mwx@gmail.com>
    
TOPIC: 'reload_config'
cwd: $HOME/humanoid/src/motion/config_watchdog
Watching directory: ../dmotion/config
Launched by dmotion/launch/dmotion.launch
Watch config directory, call `rosparam load` and publish msg when file is changed.
"""
import subprocess
import rospy
import os
from std_msgs.msg import String
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

PATH = '../dmotion/config/'
CONFIG = [
    'motion.yml',
    'motor.yml'
]


class Handler(FileSystemEventHandler):
    def __init__(self):
        super(Handler, self).__init__()
        self.pub = rospy.Publisher('/dmotion/reload_config', String, queue_size=1)
        rospy.init_node('config_watchdog')
        rospy.loginfo("%s" % os.getcwd())

    def on_modified(self, event):
        if not event.is_directory:
            _, filename = os.path.split(event.src_path)
            if filename.split('.')[-1] in ['yml', 'yaml']:
                self.pub.publish('')
                for cfg in CONFIG:
                    subprocess.call(['rosparam', 'load', '%s/%s' % (PATH, cfg)])

                # subprocess.call(['rosparam', 'get', '/dmotion/robot/diffv'])  # debug
                rospy.loginfo('config changed, published reload msg')
            else:
                rospy.logwarn('%s not yaml file' % event.src_path)


class ConfigServer(object):
    def start_watch(self):
        event_handler = Handler()
        observer = Observer()
        observer.schedule(event_handler, path=PATH, recursive=True)
        observer.start()
        rospy.spin()


if __name__ == '__main__':
    try:
        server = ConfigServer()
        server.start_watch()
    except rospy.ROSInterruptException:
        pass
