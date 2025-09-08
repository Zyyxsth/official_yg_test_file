

#!/usr/bin/env python3
import rospy
from motion_msgs.msg import MotionCtrl
from motion_msgs.msg import Diablo_Ctrl
import datetime


def generMsgs(forward=None, left=None, roll=None, up=None,
              pitch=None, mode_mark=False, height_ctrl_mode=None,
              pitch_ctrl_mode=None, roll_ctrl_mode=None, stand_mode=None,
              jump_mode=False, dance_mode=None):
    ctrlMsgs = MotionCtrl()
    ctrlMsgs.mode_mark = mode_mark
    ctrlMsgs.mode.jump_mode = jump_mode
    if dance_mode is not None:
        ctrlMsgs.mode.split_mode = dance_mode
    if forward is not None:
        ctrlMsgs.value.forward = forward
    if left is not None:
        ctrlMsgs.value.left = left
    if pitch is not None:
        ctrlMsgs.value.pitch = pitch
    if roll is not None:
        ctrlMsgs.value.roll = roll
    if up is not None:
        ctrlMsgs.value.up = up
    if height_ctrl_mode is not None:
        ctrlMsgs.mode.height_ctrl_mode = height_ctrl_mode
    if pitch_ctrl_mode is not None:
        ctrlMsgs.mode.pitch_ctrl_mode = pitch_ctrl_mode
    if roll_ctrl_mode is not None:
        ctrlMsgs.mode.roll_ctrl_mode = roll_ctrl_mode
    if stand_mode is not None:
        ctrlMsgs.mode.stand_mode = stand_mode
    return ctrlMsgs


def callback(data):
    global teleop_cmd
    ctrlMsgs = generMsgs(forward=data.speed, left=data.omega, up=1.0)
    teleop_cmd.publish(ctrlMsgs)


def main():
    global teleop_cmd
    rospy.init_node('diablo_teleop_node', anonymous=True)
    teleop_cmd = rospy.Publisher('diablo/MotionCmd', MotionCtrl, queue_size=2)
    rospy.Subscriber('/target/planning/adjust_yaw', Diablo_Ctrl, callback)
    rospy.spin()


if __name__ == "__main__":
    main()