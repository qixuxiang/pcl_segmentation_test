#!/usr/bin/env python
"""
usage: ./sync.py [robotId]
1 for robot 1, and lan
11 for wireless
"""
import os
import sys

def help():
    print 'usage:', sys.argv[0], ' xxx@xxx'

    
def main():
    if len(sys.argv) != 2:
        usage()
        return

    robot = sys.argv[1]

    cmd = "rsync -avz --exclude '.git' --exclude 'dconfig' --exclude 'devel' --exclude 'build' --exclude 'cmake-build-debug' --exclude '.idea' ~/humanoid {}:~/".format(robot)

    print cmd

    os.system(cmd)


if __name__ == '__main__':
    main()
