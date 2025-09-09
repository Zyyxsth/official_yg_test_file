#pragma once

#include <chrono>
// #include "rclcpp/rclcpp.hpp"
#include <ros/ros.h>
#include "motion_msgs/RobotStatus.h"
// #include <builtin_interfaces/msg/time.hpp>
#include "diablo_utils/diablo_tools/osdk_vehicle.hpp"

class diablo_body_state_publisher
{
private:

    ros::Timer                                                timer_;
    DIABLO::OSDK::Vehicle*                                                     vehicle;
    ros::NodeHandle&                                                   node_ptr;
    motion_msgs::RobotStatus                                     robot_state_msg_;
    ros::Time                                robot_state_timestamp;
    ros::Publisher robot_state_Publisher_;
    
public:
    diablo_body_state_publisher(ros::NodeHandle& node_ptr,DIABLO::OSDK::Vehicle* vehicle);
    ~diablo_body_state_publisher(){}
    void body_pub_init(void);
    void lazyPublisher(void);
};


