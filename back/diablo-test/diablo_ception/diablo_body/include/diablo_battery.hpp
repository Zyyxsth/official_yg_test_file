#pragma once

#include <chrono>
#include "ros/ros.h"
#include "motion_msgs/Battery.h"
#include "diablo_utils/diablo_tools/osdk_vehicle.hpp"

class diablo_battery_publisher
{
private:
    ros::Timer timer_;
    DIABLO::OSDK::Vehicle *vehicle;
    ros::NodeHandle &node_ptr;
    motion_msgs::Battery battery_msg_;
    ros::Time battery_timestamp;
    ros::Publisher battery_publisher_;

public:
    diablo_battery_publisher(ros::NodeHandle& node_ptr, DIABLO::OSDK::Vehicle *vehicle);
    ~diablo_battery_publisher() {}
    void battery_pub_init(void);
    void lazyPublisher(void);
};