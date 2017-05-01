#!/usr/bin/env python
import rospy
import os
import sys
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler


def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

CONFIG_FOLDER = '../config/'
CONFIG = [
    'motion.yml',
    'motor,yml'
]

class MyHandler(FileSystemEventHandler):
    def __init__(self, config, node):
        super(MyHandler, self).__init__()
        self.config = config
        self.node = node

    def on_modified(self, event):
        if not event.is_directory:
            try:
                rospy.loginfo('new event %s' % event.src_path)

                _, filename = os.path.split(event.src_path)
                if filename.split('.')[-1] != 'yml':
                    rospy.logwarn('not yml')
                # publish
            except:
                pass

class ConfigServer(object):
    def load(self):
        for cfg in CONFIG:
            os.system("rosparam load %s" % cfg)

    def start_watch(self):
        event_handler = MyHandler()
        observer = Observer()
        observer.schedule(event_handler, path=CONFIG_FOLDER, recursive=True)
        observer.start()
        rospy.spin()




if __name__ == '__main__':
    try:
        server = ConfigServer()
        server.start_watch()
    except rospy.ROSInterruptException:
        pass