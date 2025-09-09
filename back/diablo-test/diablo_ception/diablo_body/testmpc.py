#!/usr/bin/env python3
from http.client import OK
import rospy
import time
import sys
import tty
import termios
import threading
#from rclpy.node import Node
from motion_msgs.msg import MotionCtrl
from motion_msgs.msg import Diablo_Ctrl
print("start now!")
import datetime


ctrlMsgs = MotionCtrl()


cmd_h=0.7
mpc_cb=0


def generMsgs(forward=None,left=None,roll=None,up=None,
                pitch=None,mode_mark=False,height_ctrl_mode = None,
                pitch_ctrl_mode = None,roll_ctrl_mode = None,stand_mode = None,
                jump_mode = False,dance_mode = None):
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
    
def callback(data):
    global cmd_h
    global mpc_cb
    ground_Z=0
    mpc_cb=1
    # print("mpc_cb", mpc_cb)
    delta_Z=data.height-ground_Z
    generMsgs(forward=data.speed)
    generMsgs(left=data.omega)
    # print("delta_Z", delta_Z)
    # print("mpc runing")
    # 获取当前日期和时间
    # now = datetime.datetime.now()

    # # 格式化输出日期和时间
    # formatted_time = now.strftime("%Y-%m-%d %H:%M:%S")

    # print("当前时间是:", formatted_time)
    # cmd_h=0.3
    # if delta_Z>0.5:
    # 	cmd_h=1
    # elif cmd_h<0.3:
    # 	cmd_h=0
    # else:
    # 	cmd_h=(delta_Z-0.3)/0.2

    generMsgs(up=cmd_h)

# def callback2(data):
#     global teleop_cmd
#     ctrlMsgs = generMsgs(forward=data.speed, left=data.omega, up=1.0)
#     teleop_cmd.publish(ctrlMsgs)
     
def spin():
    rospy.spin() 

def main(args=None):
    global ctrlMsgs
    global cmd_h
    global mpc_cb
    print("get msg1")
    teleop_cmd = rospy.Publisher('diablo/MotionCmd', MotionCtrl, queue_size=2)
    print("get msg2")
    rospy.init_node('diablo_teleop_node', anonymous=True)
    print("get msg3")
    mpc_sub = rospy.Subscriber('/target/control_ugv/cmd_dia', Diablo_Ctrl, callback)
    print("get msg4")
    # rospy.Subscriber('/adjust_yaw', Diablo_Ctrl, callback2)
    print("get msg4.1")
    t1 =threading.Thread(target=spin)
    print("get msg5")
    #t1.setDaemon(True)
    t1.start()
    times=0
    while not rospy.is_shutdown():
        ctrlMsgs.mode.jump_mode = False
        ctrlMsgs.value.up=cmd_h
        # print('mpc_cb_times',times)
        # print('cmd_h',cmd_h)
        
        if mpc_cb==1 :
            times=times+1
            ctrlMsgs.mode_mark = False
            ctrlMsgs.mode.split_mode = False
            # print('cmd_h',cmd_h)
            teleop_cmd.publish(ctrlMsgs)
            mpc_cb=0
        else:
            # print("here")
            ctrlMsgs.mode_mark = False
            ctrlMsgs.mode.split_mode = False
            ctrlMsgs.value.forward = 0.0
            ctrlMsgs.value.left = 0.0
            # print('cmd_h',cmd_h)
            teleop_cmd.publish(ctrlMsgs)
            
        if ctrlMsgs.mode.jump_mode == True :
            print("jump_mode ",ctrlMsgs.mode.jump_mode)  
        time.sleep(0.04)

    

main()


