#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Vector3
from dmotion.msg import ActionCmd
from dvision.msg import SaveImg
import sys, select, termios, tty, time
msg = """
Start getting image and Publishing to Motion!
---------------------------------------------
CTRL-C to quit
"""

MIN_PLAY_PITCH = 0
MAX_PLAY_PITCH = 70
MIN_PLAY_YAW = -90
MAX_PLAY_YAW = 90
PLAY_STEP = 15
diff_pitch = [i for i in range(MIN_PLAY_PITCH, MAX_PLAY_PITCH, PLAY_STEP)]
diff_yaw = [i for i in range(-MAX_PLAY_YAW, MAX_PLAY_YAW, PLAY_STEP)]

pitch_yaw = []

for i in range(len(diff_pitch)):
    for j in range(len(diff_yaw)):
        pitch_iter = diff_pitch[i]
        if i%2 == 0 :
            yaw_iter = diff_yaw[-j - 1]
        else:
            yaw_iter = diff_yaw[j]
        pitch_yaw.append((pitch_iter, yaw_iter))



def main():
    cmd = ActionCmd()
    # cmd.cmd_head = Vector3(0, picth, yaw)
    cmd.cmd_head_speed = Vector3(0, 1, 1)
    save_img = SaveImg()
    save_img.IsSaveImg = True
    play_cnt = 0
    rospy.init_node('getImg_node', anonymous=True)
    pub_action_cmd = rospy.Publisher('humanoid/ActionCommand', ActionCmd, queue_size = 1)
    pub_save_img = rospy.Publisher('humanoid/SaveImg', SaveImg, queue_size = 1)
    time.sleep(5)
    rate = rospy.Rate(0.5)

    rospy.loginfo('Strat Getting Image')

    print msg

    while not rospy.is_shutdown():
        pitch_now = pitch_yaw[play_cnt][0]
        yaw_now = pitch_yaw[play_cnt][1]

        cmd.gait_type = ActionCmd.CROUCH
        cmd.cmd_head = Vector3(0, pitch_now, yaw_now)
        pub_action_cmd.publish(cmd)
        # time.sleep(1)
        pub_save_img.publish(save_img)
        rate.sleep()
        print(pitch_now, yaw_now)


        play_cnt += 1
        if play_cnt >= len(pitch_yaw):
            play_cnt = 0

if __name__ == '__main__':
    main()
