#!/usr/bin/env python
"""
usage: ./sync.py [robotId]
1 for robot 1, and lan
11 for wireless
"""
import os
import sys

username = 'zjudancer'
passwd = "'"

wireless_ipbase = '192.168.0.21'
lan_ipbase = '192.168.100.'
robot_num = -1

def help():
    print 'usage:', sys.argv[0], ' <bot number>'

    
def main():
    if len(sys.argv) != 2:
        usage()
        return

    num = int(sys.argv[1])
    

    ipbase = wireless_ipbase 
    if num > 10:
        num = num % 10
        ipbase = lan_ipbase

    cmd = "rsync -avz --exclude '.git' --exclude 'dconfig' --exclude 'devel' --exclude 'build' --exclude 'cmake-build-debug' --exclude '.idea' ~/humanoid {}@{}{}:~/".format(username, ipbase, num)

    print cmd

    os.system(cmd)


if __name__ == '__main__':
    main()
