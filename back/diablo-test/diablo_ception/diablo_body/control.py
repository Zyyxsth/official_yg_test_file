#!/usr/bin/env python3
import rospy
import time
from motion_msgs.msg import MotionCtrl
from motion_msgs.msg import Diablo_Ctrl

print("start now!")

# 用于存储控制消息
ctrlMsgs = MotionCtrl()

# 用于控制高度的参数
cmd_h = 1.0

# 用于标志是否收到MPC回调
mpc_cb = 0

# 用于标志是否激活旋转模式
rotate_enabled = False

def generMsgs(forward=None, left=None, roll=None, up=None,
              pitch=None, mode_mark=False, height_ctrl_mode=None,
              pitch_ctrl_mode=None, roll_ctrl_mode=None, stand_mode=None,
              jump_mode=False, dance_mode=None):
    """
    生成控制消息
    """
    global ctrlMsgs
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

def mpc_callback(data):
    """
    MPC控制回调
    """
    global cmd_h, mpc_cb
    ground_Z = 0
    mpc_cb = 1
    print("mpc_cb", mpc_cb)
    delta_Z = data.height - ground_Z
    generMsgs(forward=data.speed)
    generMsgs(left=data.omega)
    print("delta_Z", delta_Z)
    generMsgs(up=cmd_h)

def rotation_callback(data):
    """
    旋转控制回调
    """
    global rotate_enabled, ctrlMsgs
    rotate_enabled = True
    ctrlMsgs = generMsgs(forward=data.speed, left=data.omega, up=1.0)
    teleop_cmd.publish(ctrlMsgs)

def main(args=None):
    """
    主函数
    """
    global ctrlMsgs, cmd_h, mpc_cb, rotate_enabled, teleop_cmd
    print("get msg1")
    teleop_cmd = rospy.Publisher('diablo/MotionCmd', MotionCtrl, queue_size=2)
    print("get msg2")
    rospy.init_node('diablo_teleop_node', anonymous=True)
    print("get msg3")
    mpc_sub = rospy.Subscriber('/target/control_ugv/cmd_dia', Diablo_Ctrl, mpc_callback)
    print("get msg4")
    rotation_sub = rospy.Subscriber('/target/planning/adjust_yaw', Diablo_Ctrl, rotation_callback)
    print("get msg4.1")
    times = 0
    while not rospy.is_shutdown():
        ctrlMsgs.mode.jump_mode = False
        ctrlMsgs.value.up = cmd_h
        if mpc_cb == 1 and not rotate_enabled:
            times += 1
            ctrlMsgs.mode_mark = False
            ctrlMsgs.mode.split_mode = False
            teleop_cmd.publish(ctrlMsgs)
            mpc_cb = 0
        elif rotate_enabled:
            teleop_cmd.publish(ctrlMsgs)
            rotate_enabled = False  # 重置旋转标志
        else:
            ctrlMsgs.mode_mark = False
            ctrlMsgs.mode.split_mode = False
            ctrlMsgs.value.forward = 0.0
            ctrlMsgs.value.left = 0.0
            teleop_cmd.publish(ctrlMsgs)
        if ctrlMsgs.mode.jump_mode:
            print("jump_mode ", ctrlMsgs.mode.jump_mode)
        time.sleep(0.04)

if __name__ == "__main__":
    main()