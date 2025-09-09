#include <ros/ros.h>
#include "commons.h"

#include "lio_builder/lio_builder.h"
#include <tf2_ros/transform_broadcaster.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "temp_node");
    ros::NodeHandle nh;
    static tf2_ros::TransformBroadcaster br;
    ImuData imu_data;
    imu_data.topic = "/livox/imu";
    LivoxData livox_data;
    livox_data.topic = "/livox/lidar";
    MeasureGroup measure_group;
    std::string map_frame("local");
    std::string lidar_frame("body");
    ros::Subscriber imu_sub = nh.subscribe(imu_data.topic, 1000, &ImuData::callback, &imu_data);
    ros::Subscriber livox_sub = nh.subscribe(livox_data.topic, 1000, &LivoxData::callback, &livox_data);

    ros::Publisher cloud_world_pub = nh.advertise<sensor_msgs::PointCloud2>("local_cloud", 1000);
    ros::Publisher cloud_lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("body_cloud", 1000);

    ros::Rate rate(50);

    fastlio::LioParams params;
    params.align_gravity = true;
    fastlio::LIOBuilder lio_builder(params);

    while (ros::ok())
    {
        rate.sleep();
        ros::spinOnce();
        if (!measure_group.syncPackage(imu_data, livox_data))
            continue;

        lio_builder.mapping(measure_group);

        if (lio_builder.currentStatus() == fastlio::Status::INITIALIZE)
            continue;

        br.sendTransform(eigen2Transform(lio_builder.currentState().rot.toRotationMatrix(),
                                         lio_builder.currentState().pos,
                                         map_frame, lidar_frame, measure_group.lidar_time_end));
        sensor_msgs::PointCloud2 cloud_world_msg = pcl2msg(lio_builder.cloudWorld(), map_frame, livox_data.last_timestamp);
        sensor_msgs::PointCloud2 cloud_lidar_msg = pcl2msg(lio_builder.cloudUndistortedBody(), lidar_frame, livox_data.last_timestamp);
        cloud_world_pub.publish(cloud_world_msg);
        cloud_lidar_pub.publish(cloud_lidar_msg);
    }
    return 0;
}