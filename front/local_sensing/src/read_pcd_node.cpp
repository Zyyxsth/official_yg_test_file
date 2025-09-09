#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

main(int argc, char **argv) {
    ros::init(argc, argv, "read_pcd_node");
    ros::NodeHandle nh("~");

    std::string pcd_load_path, pcd_save_path;
    double pc_resolution(0.5);
    bool do_filter(false), do_save(false);
    nh.getParam("pcd_save_path", pcd_save_path);
    nh.getParam("pcd_load_path", pcd_load_path);
    nh.getParam("pc_resolution", pc_resolution);
    nh.getParam("do_filter", do_filter);
    nh.getParam("do_save", do_save);
    ROS_WARN_STREAM("pcd_save_path: " << pcd_save_path);
    ROS_WARN_STREAM("pcd_load_path: " << pcd_load_path);
    ROS_WARN_STREAM("pc_resolution: " << pc_resolution);

    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("global_map", 1);
    pcl::PointCloud<pcl::PointXYZ> cloud_input;
    pcl::io::loadPCDFile(pcd_load_path, cloud_input);  //Modify the path of your pcd file
    //Convert the cloud to ROS message

    if (do_filter) {
        pcl::PassThrough<pcl::PointXYZ> pass;          // 创建直通滤波器对象
        pass.setInputCloud(cloud_input.makeShared());  // 输入点云
        pass.setFilterFieldName("z");                  // 设置过滤字段
        pass.setFilterLimits(-1.0, 2.5);               // 设置范围
        pass.setFilterLimitsNegative(false);           // 设置字段范围内的是保留（false）还是过滤掉（true）
        pass.filter(cloud_input);

        pass.setInputCloud(cloud_input.makeShared());  // 输入点云
        pass.setFilterFieldName("x");                  // 设置过滤字段
        pass.setFilterLimits(-45.0, 45.0);             // 设置范围
        pass.setFilterLimitsNegative(false);           // 设置字段范围内的是保留（false）还是过滤掉（true）
        pass.filter(cloud_input);

        pass.setInputCloud(cloud_input.makeShared());  // 输入点云
        pass.setFilterFieldName("y");                  // 设置过滤字段
        pass.setFilterLimits(-65.0, 65.0);             // 设置范围
        pass.setFilterLimitsNegative(false);           // 设置字段范围内的是保留（false）还是过滤掉（true）
        pass.filter(cloud_input);

        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        outrem.setInputCloud(cloud_input.makeShared());
        outrem.setRadiusSearch(0.2);         // 设置搜索半径
        outrem.setMinNeighborsInRadius(50);  // 设置最少的邻点数量
        outrem.filter(cloud_input);

        pcl::VoxelGrid<pcl::PointXYZ> voxel_sampler;
        voxel_sampler.setLeafSize(pc_resolution, pc_resolution, pc_resolution);
        voxel_sampler.setInputCloud(cloud_input.makeShared());
        voxel_sampler.filter(cloud_input);
    }

    if (do_save) {
        pcl::PCDWriter writer;
        writer.write(pcd_save_path, cloud_input);
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud_input, output);
    output.header.frame_id = "world";  //this has been done in order to be able to visualize our PointCloud2 message on the RViz visualizer
    ros::Rate loop_rate(1);
    while (ros::ok()) {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}