#!/usr/bin/env python3
import rospy
import time
import threading
import datetime
from motion_msgs.msg import MotionCtrl
from motion_msgs.msg import Diablo_Ctrl

print("start now!")

# 全局变量
ctrlMsgs = MotionCtrl()
cmd_h = 1.0
mpc_cb = 0
state = "idle"  # 初始状态: idle（空闲）
last_msg_time = time.time()  # 记录最后收到消息的时间
tracking_forward = 0.0
tracking_left = 0.0
rotating_forward = 0.0
rotating_left = 0.0

# 生成 MotionCtrl 消息的函数（从原脚本复制）
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

# 轨迹跟踪回调（类似于 testmpc.py 的 callback）
def callback_tracking(data):
    global state, mpc_cb, cmd_h, last_msg_time, tracking_forward, tracking_left
    ground_Z = 0
    mpc_cb = 1
    delta_Z = data.height - ground_Z
    tracking_forward = data.speed
    tracking_left = data.omega
    # print("delta_Z", delta_Z)
    # now = datetime.datetime.now()
    # formatted_time = now.strftime("%Y-%m-%d %H:%M:%S")
    # print("当前时间是:", formatted_time)
    # cmd_h = 0.3  # 示例逻辑，您可以启用原注释部分
    # if delta_Z > 0.5:
    #     cmd_h = 1
    # elif cmd_h < 0.3:
    #     cmd_h = 0
    # else:
    #     cmd_h = (delta_Z - 0.3) / 0.2
    state = "tracking"  # 切换到 tracking 状态
    last_msg_time = time.time()  # 更新最后消息时间

# 旋转回调（类似于 rotation.py 的 callback）
def callback_rotating(data):
    global state, last_msg_time, rotating_forward, rotating_left
    rotating_forward = data.speed
    rotating_left = data.omega
    state = "rotating"  # 切换到 rotating 状态
    last_msg_time = time.time()  # 更新最后消息时间

# ROS spin 线程
def spin():
    rospy.spin()

def main():
    global ctrlMsgs, cmd_h, mpc_cb, state, last_msg_time, tracking_forward, tracking_left, rotating_forward, rotating_left
    rospy.init_node('diablo_teleop_combined', anonymous=True)
    teleop_cmd = rospy.Publisher('diablo/MotionCmd', MotionCtrl, queue_size=2)
    rospy.Subscriber('/target/control_ugv/cmd_dia', Diablo_Ctrl, callback_tracking)
    rospy.Subscriber('/target/planning/adjust_yaw', Diablo_Ctrl, callback_rotating)

    t1 = threading.Thread(target=spin)
    t1.start()

    times = 0
    timeout = 0.5  # 超时时间（秒），如果超过此时间无新消息，切换回 idle

    while not rospy.is_shutdown():
        current_time = time.time()
        if current_time - last_msg_time > timeout:
            state = "idle"  # 超时切换回 idle

        ctrlMsgs.mode.jump_mode = False
        ctrlMsgs.value.up = 1.0
        ctrlMsgs.mode_mark = False
        ctrlMsgs.mode.split_mode = False

        if state == "tracking":
            if mpc_cb == 1:
                times += 1
                print("now_state is tracking")
                generMsgs(forward=tracking_forward, left=tracking_left, up=1.0)
                teleop_cmd.publish(ctrlMsgs)
                mpc_cb = 0
            else:
                generMsgs(forward=0.0, left=0.0, up=1.0)  # 默认停止
                teleop_cmd.publish(ctrlMsgs)
        elif state == "rotating":
            print("now_state is rotatings")
            generMsgs(forward=rotating_forward, left=rotating_left, up=1.0)
            teleop_cmd.publish(ctrlMsgs)
        else:  # idle
            generMsgs(forward=0.0, left=0.0, up=1.0)  # 停止
            teleop_cmd.publish(ctrlMsgs)

        if ctrlMsgs.mode.jump_mode:
            print("jump_mode ", ctrlMsgs.mode.jump_mode)
        time.sleep(0.04)  # 25 Hz

if __name__ == "__main__":
    main()