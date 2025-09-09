#include "diablo_battery.hpp"

using namespace std::chrono;

void diablo_battery_publisher::battery_pub_init(void)
{
    battery_publisher_ = this->node_ptr.advertise<motion_msgs::Battery>("diablo_battery", 10);
    timer_ = this->node_ptr.createTimer(ros::Duration(0.1),std::bind(&diablo_battery_publisher::lazyPublisher, this));
    this->vehicle->telemetry->configTopic(DIABLO::OSDK::TOPIC_POWER, OSDK_PUSH_DATA_10Hz);
    this->vehicle->telemetry->configUpdate();
    
}

void diablo_battery_publisher::lazyPublisher(void)
{
    if (battery_publisher_.getNumSubscribers() > 0)
    {
       bool battery_pub_mark = false;

       if(this->vehicle->telemetry->newcome & 0x02)
       {
        battery_pub_mark = true;
        battery_msg_.battery_voltage =this ->vehicle->telemetry->power.voltage;
        battery_msg_.battery_current = this ->vehicle->telemetry->power.current;
        battery_msg_.battery_capacitor_energy = this ->vehicle->telemetry->power.capacitor_energy;
        battery_msg_.battery_power_percent = this ->vehicle->telemetry->power.power_percent;

        this->vehicle->telemetry->eraseNewcomeFlag(0xFD);
        }
        if(battery_pub_mark)
        {
            // battery_timestamp = ros::Time::now();
            // battery_msg_.header.stamp = battery_timestamp;
            // battery_msg_.header.frame_id = "diablo_robot";
            battery_publisher_.publish(battery_msg_);
            battery_pub_mark = false;

        }
    }
}

diablo_battery_publisher::diablo_battery_publisher(ros::NodeHandle& node,DIABLO::OSDK::Vehicle* vehicle)
:node_ptr(node)
{
    this->vehicle = vehicle;
    
}