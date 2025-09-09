#!/usr/bin/env python3
from http.client import OK
import rospy
import time
import sys
import tty
import termios
import threading
from nav_msgs.msg import Odometry
#from rclpy.node import Node
from motion_msgs.msg import MotionCtrl

print("Teleoptest start now!")
print("Press '`' to exit!")

keyQueue = []
ctrlMsgs = MotionCtrl()
old_setting = termios.tcgetattr(sys.stdin)

odom_msg = Odometry()

def readchar():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def getKeyBoard():
    global keyQueue
    while True:
        c = readchar()
        keyQueue.append(c)


t1 =threading.Thread(target=getKeyBoard)
t1.setDaemon(True)
t1.start()


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

def Odomcallback(data):
    global odom_msg
    odom_msg = data

def write_odom():
    print("write_odometry_ready!!!!!!!")
    global odom_msg
    print(odom_msg.pose.pose.position)
    with open("odometry_data2222.txt", "a") as file:
        print("open file-------------")
        # 获取时间戳
        #timestamp = odom_msg.header.stamp.to_sec()
        # 获取位姿数据
        #position = odom_msg.pose.pose.position
        #orientation = odom_msg.pose.pose.orientation
        # 将数据格式化为字符串
        data = "{},{},{},{},{},{},{},{}\n".format(
            odom_msg.header.stamp.to_sec(),
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            odom_msg.pose.pose.position.z,
            # 0.0,0.0, 0.0,1.0;
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w
        )
        # 写入文件
        file.write(data)
    print("close file !!")    


def main(args=None):
    global ctrlMsgs,odom_msg
   # global pos_x,pos_y,pos_z,ori_x,ori_y,ori_z,ori_w
    
    teleop_cmd = rospy.Publisher('diablo/MotionCmd', MotionCtrl, queue_size=10)
    mpc_sub = rospy.Subscriber('/new_odom', Odometry, Odomcallback)
    rospy.init_node('diablo_teleop_node', anonymous=True)

    while True:
        if len(keyQueue) > 0:
            key = keyQueue.pop(0)
            print("key ",key)
            if key == 'w':
                generMsgs(forward=0.5)
            elif key == 's':
                generMsgs(forward=-1.0)
            elif key == 'a':
                generMsgs(left=1.0)
            elif key == 'd':
                generMsgs(left=-1.0)
            elif key == 'e':
                generMsgs(roll=0.1)
            elif key == 'q':
                generMsgs(roll=-0.1)
            elif key == 'r':
                generMsgs(roll=0.0)

            elif key == 'h':
                generMsgs(up = 0.1)
            elif key == 'j':
                generMsgs(up = 1.0)
            elif key == 'k':
               generMsgs(up = 0.5)
            elif key == 'l':
               generMsgs(up = 0.0)
                
            elif key == 'u':
                generMsgs(pitch = 0.5)
            elif key == 'i':
                generMsgs(pitch = 0.0)
            elif key == 'o':
                generMsgs(pitch = -0.5)

            elif key == 'v':
                generMsgs(mode_mark=True,height_ctrl_mode=True)
            elif key == 'b':
                generMsgs(mode_mark=True,height_ctrl_mode=False)
            elif key == 'n':
                generMsgs(mode_mark=True,pitch_ctrl_mode=True)
            elif key == 'm':
                generMsgs(mode_mark=True,pitch_ctrl_mode=False)

            elif key == 'z':
                generMsgs(mode_mark=True,stand_mode=True, jump_mode=False)
                teleop_cmd.publish(ctrlMsgs)
                generMsgs(up=0.5)
                teleop_cmd.publish(ctrlMsgs)
            elif key == 'x':
                generMsgs(mode_mark=True,stand_mode=False)
                teleop_cmd.publish(ctrlMsgs)
            elif key == 'y':
                generMsgs(mode_mark=True,jump_mode=True)
                teleop_cmd.publish(ctrlMsgs)
            elif key == 'f':
                generMsgs(mode_mark=True,dance_mode=True)
                teleop_cmd.publish(ctrlMsgs)
            elif key == 'g':
                generMsgs(mode_mark=True,dance_mode=False)
                teleop_cmd.publish(ctrlMsgs)

 #-------------------------------t记录当前位姿到txt中-----------------------------------------------------------
            elif key == 't':
                write_odom()
 # -----------------------------------------------------------------------------------------          
            elif key == '`':     
                break
        else:
            ctrlMsgs.mode_mark = False
            ctrlMsgs.mode.split_mode = False
            ctrlMsgs.value.forward = 0.0
            ctrlMsgs.value.left = 0.0
        if ctrlMsgs.mode.jump_mode == True :
            print("jump_mode ",ctrlMsgs.mode.jump_mode)  
        #print("up ",ctrlMsgs.value.up)  
        teleop_cmd.publish(ctrlMsgs)
        time.sleep(0.04)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_setting)
    print('exit!')
    

main()


