#!/usr/bin/env python3
import rospy
import socket                            
import json
import threading
from nav_msgs.msg import Odometry
from motion_msgs.msg import MotionCtrl
import subprocess
import sys
import os
sys.path.append("/home/hialb/siyi/siyi_sdk")

from siyi_sdk import SIYISDK

def sendmsg(message, target_ip='10.1.1.133', target_port=40000):
    try:
        # 创建 UDP 套接字
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 发送消息
        sock.sendto(message.encode('utf-8'), (target_ip, target_port))
    except Exception as e:
        print(f"发送消息时出错: {e}")
    finally:
        # 关闭套接字
        sock.close()


print("Teleoptest start now!")
print("Press Ctrl+C to exit!")

ctrlMsgs = MotionCtrl()
odom_msg = Odometry()

# UDP 接收端口
UDP_PORT = 12345
# 创建 UDP 套接字
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    sock.bind(('', UDP_PORT))
    print(f"Successfully connected to UDP port {UDP_PORT}. Waiting for data...")
    sendmsg("Successfully connected to UDP port")
except Exception as e:
    print(f"Failed to bind to UDP port {UDP_PORT}: {e}")
    exit(1)


def generMsgs(forward=None, left=None, roll=None, up=None,
              pitch=None, mode_mark=False, height_ctrl_mode=None,
              pitch_ctrl_mode=None, roll_ctrl_mode=None, stand_mode=None,
              jump_mode=False, dance_mode=None):
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


def write_odom(cam):
    print("write_odometry_ready!!!!!!!")
    global odom_msg
    print(odom_msg.pose.pose.position)
    yaw, pitch, _ = cam.getAttitude()
    with open("odometry_data_test.txt", "a+") as file:
        print("open file-------------")
        data = "{},{},{},{},{},{},{},{},{},{}\n".format(
            odom_msg.header.stamp.to_sec(),
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            odom_msg.pose.pose.position.z,
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w,
            yaw,
            pitch
        )
        file.write(data)
    string = "odom data have been saved"
    sendmsg(string)
    print("close file !!")


def receive_udp():
    cam = SIYISDK(server_ip="192.168.1.25", port=37260)
    target_pitch_deg = 0
    target_yaw_deg = 0
    if not cam.connect():
        print("No connection ")
        sendmsg("No connection")
    cam.requestSetAngles(0 ,0)
    v_speed = 50
    tv_speed = 50

    autocheck_pid = 0
 
    global ctrlMsgs
    while True:
        try:
            # 接收 UDP 数据
            data, addr = sock.recvfrom(1024)
            try:
                # 解析 JSON 数据
                json_data = json.loads(data.decode())
                key = json_data.get('key')
                v_speed = json_data.get('speed')
                tv_speed = json_data.get('turn_speed')
                v_speed = v_speed*0.01*0.8
                tv_speed = tv_speed*0.01*0.8
                if v_speed<0.3:
                    v_speed = 0.3
                if tv_speed < 0.3:
                    tv_speed = 0.3


                print("Received key:", key)

                if key == 'w':
                    generMsgs(forward=v_speed)
                elif key == 's':
                    generMsgs(forward=0- v_speed)
                elif key == 'stop':
                    generMsgs(forward=0)
                elif key == 'a':
                    generMsgs(left=tv_speed)
                elif key == 'stop_t':
                    generMsgs(left=0)
                elif key == 'd':
                    generMsgs(left=0 -tv_speed)
                elif key == 'e':
                    generMsgs(roll=0.1)
                elif key == 'q':
                    generMsgs(roll=-0.1)
                elif key == 'r':
                    generMsgs(roll=0.0)
                elif key == 'h':
                    generMsgs(up=0)
                elif key == 'j':
                    generMsgs(up=1.0)
                elif key == 'k':
                    generMsgs(up=0.5)
                elif key == 'l':
                    generMsgs(up=0.0)
                elif key == 'u':
                    generMsgs(pitch=0.5)
                elif key == 'i':
                    generMsgs(pitch=0.0)
                elif key == 'o':
                    generMsgs(pitch=-0.5)
                elif key == 'v':
                    generMsgs(mode_mark=True, height_ctrl_mode=True)
                elif key == 'b':
                    generMsgs(mode_mark=True, height_ctrl_mode=False)
                elif key == 'n':
                    generMsgs(mode_mark=True, pitch_ctrl_mode=True)
                elif key == 'm':
                    generMsgs(mode_mark=True, pitch_ctrl_mode=False)
                elif key == 'z':
                    generMsgs(mode_mark=True, stand_mode=True)
                    teleop_cmd.publish(ctrlMsgs)
                    generMsgs(up=1.0)
                    teleop_cmd.publish(ctrlMsgs)
                    sendmsg("Robot stand up")
                elif key == 'x':
                    generMsgs(mode_mark=True, stand_mode=False)
                    teleop_cmd.publish(ctrlMsgs)
                    sendmsg("Robot sit down")
                elif key == 'c':
                    generMsgs(mode_mark=True, jump_mode=True)
                    teleop_cmd.publish(ctrlMsgs)
                elif key == 'f':
                    generMsgs(mode_mark=True, dance_mode=True)
                    teleop_cmd.publish(ctrlMsgs)
                elif key == 'g':
                    generMsgs(mode_mark=True, dance_mode=False)
                    teleop_cmd.publish(ctrlMsgs)
                elif key == 'checkpoint':
                    write_odom(cam)
                elif key == 'siyi_right':
                    
                    #cam.requestHardwareID() # Important to get the angles limits defined in cameras.py
                    
                    target_yaw_deg += 5
                    target_pitch_deg += 0
                    cam.requestSetAngles(target_yaw_deg, target_pitch_deg)
                    print("Attitude (yaw,pitch,roll) eg:", cam.getAttitude())
                    

                    print("Done and closing...")
                    #cam.disconnect()
                elif key == 'siyi_down':
                    
                    target_pitch_deg += 5
                    cam.requestSetAngles(target_yaw_deg, target_pitch_deg)
                    print("Attitude (yaw,pitch,roll) eg:", cam.getAttitude())
                    
                elif key == 'siyi_up':
                    
                    target_pitch_deg -= 5
                    cam.requestSetAngles(target_yaw_deg, target_pitch_deg)
                    print("Attitude (yaw,pitch,roll) eg:", cam.getAttitude())
                elif key == 'siyi_left':
                    
                     #cam.requestHardwareID() # Important to get the angles limits defined in cameras.py
                    #target_yaw_deg ,siyi_pitch, siyi_roll= cam.getAttitude()
                    target_yaw_deg -= 5
                    target_pitch_deg += 0
                    cam.requestSetAngles(target_yaw_deg, target_pitch_deg)
                    print("Attitude (yaw,pitch,roll) eg:", cam.getAttitude())
                    
                    print("Done and closing...")
                    #cam.disconnect()
                elif key == "siyi_reset":
                     cam.requestSetAngles(0, 0)
                     print("Attitude (yaw,pitch,roll) eg:", cam.getAttitude())
                elif key == 'autocheck':
                    #cam.disconnect()
                    
                    os.chmod('/home/hialb/Elastic-Tracker-main/kkk.sh', 0o700)
                    cmd = ['gnome-terminal','--','bash','-c',f'bash "kkk.sh";exec bash']
                    wind = subprocess.Popen(cmd)
                    autocheck_pid = wind.pid
                    sendmsg("The self-checking is being executed")
                elif key == 'close':
                    
                    #os.chmod('/home/hialb/Elastic-Tracker-main/closeSetup.sh', 0o700)
                    cmd = ['gnome-terminal','--','bash','-c',f'kill {autocheck_pid};exec bash']
                    subprocess.Popen(cmd)
                    sendmsg("The self-checking is being executed")
                elif key == '`':
                    # 可根据需要添加退出逻辑，这里简单打印信息
                    print('Exit command received!')

            except json.JSONDecodeError:
                print("Failed to decode JSON data.")
        except Exception as e:
            print(f"Error receiving UDP data: {e}")


# 启动 UDP 接收线程
t1 = threading.Thread(target=receive_udp)
t1.setDaemon(True)
t1.start()


def main(args=None):
    global ctrlMsgs, odom_msg

    teleop_cmd = rospy.Publisher('diablo/MotionCmd', MotionCtrl, queue_size=10)
    mpc_sub = rospy.Subscriber('/new_odom', Odometry, Odomcallback)
    rospy.init_node('diablo_teleop_node', anonymous=True)

    rate = rospy.Rate(25)  # 25 Hz
    try:
        while not rospy.is_shutdown():
            if ctrlMsgs.mode.jump_mode:
                print("jump_mode ", ctrlMsgs.mode.jump_mode)
            # 发布控制消息
            teleop_cmd.publish(ctrlMsgs)
            rate.sleep()
    except KeyboardInterrupt:
        print('exit!')


if __name__ == '__main__':
    main()