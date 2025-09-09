#pragma once

#include <chrono>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
// #include <sensor_msgs/Imu.hpp>
// #include <sensor_msgs

#include "diablo_utils/diablo_tools/osdk_vehicle.hpp"

class diablo_imu_publisher
{
private:
    ros::Timer timer_;

    DIABLO::OSDK::Vehicle *vehicle;
    sensor_msgs::Imu imu_msg_;
    ros::NodeHandle &node_ptr;
    ros::Time                                imu_timestamp;

    ros::Publisher imu_Publisher_;
    // ros::Publisher euler_Publisher_;

public:
    diablo_imu_publisher(ros::NodeHandle& node_ptr, DIABLO::OSDK::Vehicle *vehicle);
    // void toEulerAngle(void);
    ~diablo_imu_publisher() {}
    void imu_pub_init(void);
    void lazyPublisher(void);
};
