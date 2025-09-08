#include <iostream>
#include <string>
#include <cstdlib>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include "diablo_sdk/Diablo_Ctrl.h"

// 预设的odom信息
nav_msgs::Odometry preset_odom;
// 当前的odom信息
nav_msgs::Odometry current_odom;

ros::Publisher pos_cmd_pub_;

ros::Subscriber odom_sub_;

nav_msgs::Odometry odom_msg_;

bool odom_received_ = false;

Eigen::Vector3d odom_p;
Eigen::Vector4d odom_q;

// 角度阈值
const double ANGLE_THRESHOLD = 5.0; // 单位：度

const double ANGULAR_VELOCITY_VALUE = 0.5;

// 四元数转欧拉角
void quaternionToEuler(const geometry_msgs::Quaternion& quat, double& roll, double& pitch, double& yaw) {
    double w = quat.w;
    double x = quat.x;
    double y = quat.y;
    double z = quat.z;

    double norm = std::sqrt(w * w + x * x + y * y + z * z);
    if (norm > 1e-6) {
        w /= norm;
        x /= norm;
        y /= norm;
        z /= norm;
    }

    // 计算roll
    roll = std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));

    // 计算pitch
    pitch = std::asin(2 * (w * y - z * x));

    // 计算yaw
    yaw = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));

    std::cout << "yaw = " << yaw << std::endl;
}

// 计算插值并判断是否小于阈值
bool calculateInterpolation(double& roll_interpolation, double& pitch_interpolation) {
    double preset_roll, preset_pitch, preset_yaw;
    double current_roll, current_pitch, current_yaw;

    // 将预设的四元数转换为欧拉角
    quaternionToEuler(preset_odom.pose.pose.orientation, preset_roll, preset_pitch, preset_yaw);
    // 将当前的四元数转换为欧拉角
    quaternionToEuler(current_odom.pose.pose.orientation, current_roll, current_pitch, current_yaw);

    // std::cout << "preset_yaw= " << preset_yaw << " current_yaw = " << current_yaw << std::endl;

    // 计算插值
    roll_interpolation = preset_yaw - current_yaw;
    pitch_interpolation = preset_pitch - current_pitch;

    

    roll_interpolation = roll_interpolation * (180.0 / M_PI);
    pitch_interpolation = pitch_interpolation * (180.0 / M_PI);

    // std::cout << "roll = " << roll_interpolation << " pitch = " << pitch_interpolation << std::endl;


    // 判断是否小于阈值
    return std::abs(roll_interpolation) < ANGLE_THRESHOLD && std::abs(pitch_interpolation) < ANGLE_THRESHOLD;
}

// 当前odom信息回调函数
// void odom_callback(const nav_msgs::Odometry::ConstPtr& msgPtr) {
//     try {

        
//         odom_msg_ = *msgPtr;
//         std::cout << "no_problem?" << std::endl;
//         odom_msg_.pose.pose.position.z = 0.5;
//         odom_received_ = true;

//         odom_p = Eigen::Vector3d(odom_msg_.pose.pose.position.x,
//                                  odom_msg_.pose.pose.position.y,
//                                  odom_msg_.pose.pose.position.z);

//         odom_q = Eigen::Vector4d(odom_msg_.pose.pose.orientation.x,
//                                  odom_msg_.pose.pose.orientation.y,
//                                  odom_msg_.pose.pose.orientation.z,
//                                  odom_msg_.pose.pose.orientation.w);

//         current_odom.pose.pose.position.x = odom_msg_.pose.pose.position.x;
//         current_odom.pose.pose.position.y = odom_msg_.pose.pose.position.y;
//         current_odom.pose.pose.position.z = odom_msg_.pose.pose.position.z;
//         current_odom.pose.pose.orientation.x = odom_msg_.pose.pose.orientation.x;
//         current_odom.pose.pose.orientation.y = odom_msg_.pose.pose.orientation.y;
//         current_odom.pose.pose.orientation.z = odom_msg_.pose.pose.orientation.z;
//         current_odom.pose.pose.orientation.w = odom_msg_.pose.pose.orientation.w;

//         std::cout << "odom_q = " << current_odom.pose.pose.orientation.x << std::endl;
//         std::cout.flush();
//     } catch (const std::exception& e) {
//         std::cerr << "Exception in odom_callback: " << e.what() << std::endl;
//     }

//     // odom_msg_ = *msgPtr;
//     // std::cout << "no_problem?"<<std::endl;
//     // odom_msg_.pose.pose.position.z = 0.5;  //--------------------------------------------
//     // odom_received_ = true;

//     // odom_p = Eigen::Vector3d(odom_msg_.pose.pose.position.x,
//     //                          odom_msg_.pose.pose.position.y,
//     //                          odom_msg_.pose.pose.position.z);
//     // // odom_v = Eigen::Vector3d(odom_msg_.twist.twist.linear.x,
//     // //                          odom_msg_.twist.twist.linear.y,
//     // //                          odom_msg_.twist.twist.linear.z);
//     // odom_q = Eigen::Vector4d(odom_msg_.pose.pose.orientation.x,
//     //     odom_msg_.pose.pose.orientation.y,
//     //     odom_msg_.pose.pose.orientation.z,
//     //     odom_msg_.pose.pose.orientation.w);

//     // current_odom.pose.pose.position.x = odom_msg_.pose.pose.position.x;
//     // current_odom.pose.pose.position.y = odom_msg_.pose.pose.position.y;
//     // current_odom.pose.pose.position.z = odom_msg_.pose.pose.position.z;
//     // current_odom.pose.pose.orientation.x = odom_msg_.pose.pose.orientation.x;
//     // current_odom.pose.pose.orientation.y = odom_msg_.pose.pose.orientation.y;
//     // current_odom.pose.pose.orientation.z = odom_msg_.pose.pose.orientation.z;
//     // current_odom.pose.pose.orientation.w = odom_msg_.pose.pose.orientation.w;

//     // std::cout << "odom_q = " << current_odom.pose.pose.orientation.x << std::endl;
//     // std::cout.flush();
// }

void odom_callback(const nav_msgs::Odometry::ConstPtr& msgPtr) {
    try {
        // 将收到的odom信息复制到current_odom中
        current_odom = *msgPtr;
        odom_received_ = true; // 表示成功接收到odom信息

        // 打印调试信息，确认回调函数被调用
        ROS_INFO("Callback triggered: Received odometry information.");
        // // 打印一些odom信息以供调试
        // ROS_INFO("Position x: %f", current_odom.pose.pose.position.x);
        // ROS_INFO("Position y: %f", current_odom.pose.pose.position.y);
        // ROS_INFO("Orientation x: %f", current_odom.pose.pose.orientation.x);
        // ROS_INFO("Orientation y: %f", current_odom.pose.pose.orientation.y);
        // ROS_INFO("Orientation z: %f", current_odom.pose.pose.orientation.z);
        // ROS_INFO("Orientation w: %f", current_odom.pose.pose.orientation.w);
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in odom_callback: %s", e.what());
    }
}

// 循环发送命令的函数
void sendCommands(ros::Publisher& pub) {
    ros::Rate rate(5); // 设置发布频率为10Hz
    std::cout << "enter xxx" << std::endl;
    diablo_sdk::Diablo_Ctrl msg;
    while (ros::ok()) {
        ros::spinOnce();
        double roll_angular_velocity, pitch_angular_velocity;
        if (calculateInterpolation(roll_angular_velocity, pitch_angular_velocity)) {
            // 角度差值小于阈值，不再发布消息
            msg.speed = 0.0; // 根据实际情况设置
            msg.omega = 0.0;
            pub.publish(msg);
            break;
        }
        msg.speed = 0.0; // 根据实际情况设置
        // 根据roll_angular_velocity的正负来设置msg.omega
        if (roll_angular_velocity > 0) {
            msg.omega = ANGULAR_VELOCITY_VALUE;  // 右转
        } else {
            msg.omega = -ANGULAR_VELOCITY_VALUE; // 左转
        }    // 根据实际情况设置

        pub.publish(msg);

        rate.sleep(); // 按照设定的频率休眠
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_interpolation_node");
    ros::NodeHandle nh;

    // odom_sub_ = nh.subscribe<nav_msgs::Odometry>("Robot_base_odom", 10, &odom_callback, ros::TransportHints().tcpNoDelay());
    odom_sub_ = nh.subscribe("Robot_base_odom", 10, &odom_callback);
    // odom_sub_.subscribe(nh, "odom", 50);
    // std::cout << current_odom << std::endl;

    // 初始化预设odom信息，这里简单示例为固定值，你可以根据实际需求修改
    preset_odom.pose.pose.position.x = 0;
    preset_odom.pose.pose.position.y = 0;
    preset_odom.pose.pose.position.z = 0;
    preset_odom.pose.pose.orientation.x = -0.021154111865973043;
    preset_odom.pose.pose.orientation.y = 0.021765272511788843;
    preset_odom.pose.pose.orientation.z = 0.8867099389441564;
    preset_odom.pose.pose.orientation.w = 0.4613287988418278;

    pos_cmd_pub_ = nh.advertise<diablo_sdk::Diablo_Ctrl>("adjust_yaw", 10);

    sendCommands(pos_cmd_pub_);

    ros::spin();

    return 0;
}