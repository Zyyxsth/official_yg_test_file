#!/usr/bin/env python
import rospy
import socket
import json
import time
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from motion_msgs.msg import LegMotors, RobotStatus, Battery

class DiabloRemote:
    def __init__(self):
        rospy.init_node('diablo_remote', anonymous=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # self.target_ip = '10.1.1.37'
        self.target_ip = '10.1.1.133'
        self.target_port = 10240   # 订阅新话题
        self.imu_sub = rospy.Subscriber('/diablo/sensor/Imu', Imu, self.imu_callback)
        self.odom_sub = rospy.Subscriber('/new_odom', Odometry, self.odom_callback)
        self.motors_sub = rospy.Subscriber('/diablo/sensor/Motors', LegMotors, self.motors_callback)
        self.body_state_sub = rospy.Subscriber('/diablo/sensor/Body_state', RobotStatus, self.robot_status_callback)
        self.battery_sub = rospy.Subscriber('/diablo_battery', Battery, self.battery_callback)
        self.infopkg = 0

        # 初始化数据存储
        self.imu_data = None
        self.odom_data = None
        self.motors_data = None
        self.battery_data = None
        self.robot_status_data = None

    def imu_callback(self, msg):
        self.imu_data = {
            'orientation': {
                'x': msg.orientation.x,
                'y': msg.orientation.y,
                'z': msg.orientation.z,
                'w': msg.orientation.w
            },
            'angular_velocity': {
                'x': msg.angular_velocity.x,
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z
            },
            'linear_acceleration': {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z
            }
        }
        self.send_data()

    def odom_callback(self, msg):
        self.odom_data = {
            'pose': {
                'position': {
                    'x': msg.pose.pose.position.x,
                    'y': msg.pose.pose.position.y,
                    'z': msg.pose.pose.position.z
                },
                'orientation': {
                    'x': msg.pose.pose.orientation.x,
                    'y': msg.pose.pose.orientation.y,
                    'z': msg.pose.pose.orientation.z,
                    'w': msg.pose.pose.orientation.w
                }
            },
            'twist': {
                'linear': {
                    'x': msg.twist.twist.linear.x,
                    'y': msg.twist.twist.linear.y,
                    'z': msg.twist.twist.linear.z
                },
                'angular': {
                    'x': msg.twist.twist.angular.x,
                    'y': msg.twist.twist.angular.y,
                    'z': msg.twist.twist.angular.z
                }
            }
        }
        self.send_data()

    def motors_callback(self, msg):
        motors_info = []
        motor_names=[
            "left_hip","left_knee","left_wheel",
            "right_hip","right_knee","right_wheel"
        ]
        for i , motor_name in enumerate(motor_names):
            enc_rev_attr = f"{motor_name}_enc_rev"
            pos_attr = f"{motor_name}_pos"
            vel_attr = f"{motor_name}_vel"
            iq_attr = f"{motor_name}_iq"

            motor ={
                'enc_rev':getattr(msg, enc_rev_attr),
                'pos' : getattr(msg, pos_attr),
                'vel' : getattr(msg,vel_attr)
            }
            motors_info.append(motor)
        self.motors_data = motors_info
        self.send_data()


    def battery_callback(self, msg):
        self.battery_data = {
            'voltage': msg.battery_voltage,
            'current': msg.battery_current,
            'percentage': msg.battery_power_percent
        }
        self.send_data()

    def robot_status_callback(self, msg):
        self.robot_status_data = {
            'ctrl_mode_msg': msg.ctrl_mode_msg,
            'robot_mode_msg': msg.robot_mode_msg,
            'error_msg': msg.error_msg,
            'warning_msg': msg.warning_msg
        }
        self.send_data()

    def send_data(self):
    	
        all_data = {
            'imu': self.imu_data,
            'odom': self.odom_data,
            #'motors': self.motors_data,
            'battery': self.battery_data,
            #'robot_status': self.robot_status_data
        }
        all_data = {k: v for k, v in all_data.items() if v is not None}
        try:
            json_data = json.dumps(all_data).encode('utf-8')
            self.sock.sendto(json_data, (self.target_ip, self.target_port))
        
        except Exception as e:
            rospy.logerr(f"Failed to send data: {e}")

if __name__ == '__main__':
    try:
        diablo_remote = DiabloRemote()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
