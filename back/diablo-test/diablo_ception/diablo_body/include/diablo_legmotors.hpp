#pragma once

#include <chrono>
#include "ros/ros.h"
#include "motion_msgs/LegMotors.h"
// #include "motion_msgs/msg/leg_motors.hpp"
// #include <builtin_interfaces/msg/time.hpp>
#include "diablo_utils/diablo_tools/osdk_vehicle.hpp"

class diablo_motors_publisher
{
private:
    ros::Timer timer_;
    DIABLO::OSDK::Vehicle *vehicle;
    ros::NodeHandle &node_ptr;
    motion_msgs::LegMotors motors_msg_;
    ros::Time motors_timestamp;
    ros::Publisher motors_Publisher_;

public:
    diablo_motors_publisher(ros::NodeHandle& node_ptr, DIABLO::OSDK::Vehicle *vehicle);
    ~diablo_motors_publisher() {}
    void motors_pub_init(void);
    void lazyMotorsPublisher(void);
};
