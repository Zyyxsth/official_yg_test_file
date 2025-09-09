#include <thread>
#include <csignal>
#include <ros/ros.h>
#include "localizer/icp_localizer.h"
#include "lio_builder/lio_builder.h"
#include <tf2_ros/transform_broadcaster.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include "fastlio/SlamReLoc.h"
#include "fastlio/MapConvert.h"
#include "fastlio/SlamHold.h"
#include "fastlio/SlamStart.h"
#include "fastlio/SlamRelocCheck.h"

bool terminate_flag = false;

void signalHandler(int signum)
{
    std::cout << "SHUTTING DOWN LOCALIZER NODE!" << std::endl;
    terminate_flag = true;
}

struct SharedData
{
    std::mutex service_mutex;
    std::mutex main_mutex;
    bool pose_updated = false;
    bool localizer_activate = false;
    bool service_called = false;
    bool service_success = false;

    std::string map_path;
    Eigen::Matrix3d offset_rot = Eigen::Matrix3d::Identity();
    Eigen::Vector3d offset_pos = Eigen::Vector3d::Zero();
    Eigen::Matrix3d local_rot;
    Eigen::Vector3d local_pos;
    Eigen::Matrix4d initial_guess;
    Eigen::Matrix4d global_change;
    fastlio::PointCloudXYZI::Ptr cloud;

    bool reset_flag = false;
    bool halt_flag = false;
};

class LocalizerThread
{
public:
    LocalizerThread() {}

    void setSharedDate(std::shared_ptr<SharedData> shared_data)
    {
        shared_data_ = shared_data;
    }

    void setRate(double rate)
    {
        rate_ = std::make_shared<ros::Rate>(rate);
    }
    void setRate(std::shared_ptr<ros::Rate> rate)
    {
        rate_ = rate;
    }
    void setLocalizer(std::shared_ptr<fastlio::IcpLocalizer> localizer)
    {
        icp_localizer_ = localizer;
    }

    void operator()()
    {
        current_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);

        while (ros::ok())
        {
            rate_->sleep();
            if (terminate_flag)
                break;
            if (shared_data_->halt_flag)
                continue;
            if (!shared_data_->localizer_activate)
                continue;
            if (!shared_data_->pose_updated)
                continue;
            gloabl_pose_.setIdentity();
            bool rectify = false;
            Eigen::Matrix4d init_guess;
            {
                std::lock_guard<std::mutex> lock(shared_data_->main_mutex);
                shared_data_->pose_updated = false;
                init_guess.setIdentity();
                local_rot_ = shared_data_->local_rot;
                local_pos_ = shared_data_->local_pos;
                init_guess.block<3, 3>(0, 0) = shared_data_->offset_rot * local_rot_;
                init_guess.block<3, 1>(0, 3) = shared_data_->offset_rot * local_pos_ + shared_data_->offset_pos;
                pcl::copyPointCloud(*shared_data_->cloud, *current_cloud_);
            }

            if (shared_data_->service_called)
            {
                std::lock_guard<std::mutex> lock(shared_data_->service_mutex);
                shared_data_->service_called = false;
                icp_localizer_->init(shared_data_->map_path, false);
                gloabl_pose_ = icp_localizer_->multi_align_sync(current_cloud_, shared_data_->initial_guess);

                std::cout << "----------------------------call---1--------------------------" << std::endl;

                if (icp_localizer_->isSuccess())
                {
                    std::cout << "----------------------------success----------" << std::endl;
                    rectify = true;
                    shared_data_->localizer_activate = true;
                    shared_data_->service_success = true;
                }

                else
                {
                    rectify = false;
                    shared_data_->localizer_activate = false;
                    shared_data_->service_success = false;
                }
            }
            else
            {
                gloabl_pose_ = icp_localizer_->align(current_cloud_, init_guess);
                std::cout << "----------------------------call---1-2--------------------------" << std::endl;

                if (icp_localizer_->isSuccess())
                    rectify = true;
                else
                    rectify = false;
            }

            if (rectify)
            {
                std::lock_guard<std::mutex> lock(shared_data_->main_mutex);
                shared_data_->offset_rot = gloabl_pose_.block<3, 3>(0, 0) * local_rot_.transpose();
                shared_data_->offset_pos = -gloabl_pose_.block<3, 3>(0, 0) * local_rot_.transpose() * local_pos_ + gloabl_pose_.block<3, 1>(0, 3);
                shared_data_->global_change = gloabl_pose_;
                std::cout << "----------------------------call---2--------------------------" << std::endl;
                std::cout << gloabl_pose_ << std::endl;
            }
        }
    }

private:
    std::shared_ptr<SharedData> shared_data_;
    std::shared_ptr<fastlio::IcpLocalizer> icp_localizer_;
    std::shared_ptr<ros::Rate> rate_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud_;
    Eigen::Matrix4d gloabl_pose_;
    Eigen::Matrix3d local_rot_;
    Eigen::Vector3d local_pos_;
};

class LocalizerROS
{
public:
    LocalizerROS(tf2_ros::TransformBroadcaster &br, std::shared_ptr<SharedData> shared_data) : shared_date_(shared_data), br_(br)
    {
        initParams();
        initSubscribers();
        initPublishers();
        initServices();
        lio_builder_ = std::make_shared<fastlio::LIOBuilder>(lio_params_);
        icp_localizer_ = std::make_shared<fastlio::IcpLocalizer>(localizer_params_.refine_resolution,
                                                                 localizer_params_.rough_resolution,
                                                                 localizer_params_.refine_iter,
                                                                 localizer_params_.rough_iter,
                                                                 localizer_params_.thresh);
        icp_localizer_->setSearchParams(localizer_params_.xy_offset, localizer_params_.yaw_offset, localizer_params_.yaw_resolution);
        localizer_loop_.setRate(loop_rate_);
        localizer_loop_.setSharedDate(shared_data);
        localizer_loop_.setLocalizer(icp_localizer_);
        localizer_thread_ = std::make_shared<std::thread>(std::ref(localizer_loop_));
    }

    void initParams()
    {
        nh_.param<std::string>("map_frame", global_frame_, "map");
        nh_.param<std::string>("local_frame", local_frame_, "local");
        nh_.param<std::string>("body_frame", body_frame_, "body");
        nh_.param<std::string>("body_frame", Robot_base_frame_, "world");
        nh_.param<std::string>("imu_topic", imu_data_.topic, "/livox/imu");
        nh_.param<std::string>("livox_topic", livox_data_.topic, "/livox/lidar");
        nh_.param<bool>("publish_map_cloud", publish_map_cloud_, false);
        double local_rate, loop_rate;
        nh_.param<double>("local_rate", local_rate, 20.0);
        nh_.param<double>("loop_rate", loop_rate, 1.0);
        local_rate_ = std::make_shared<ros::Rate>(local_rate);
        loop_rate_ = std::make_shared<ros::Rate>(loop_rate);

        nh_.param<double>("lio_builder/det_range", lio_params_.det_range, 100.0);
        nh_.param<double>("lio_builder/cube_len", lio_params_.cube_len, 500.0);
        nh_.param<double>("lio_builder/resolution", lio_params_.resolution, 0.1);
        nh_.param<double>("lio_builder/move_thresh", lio_params_.move_thresh, 1.5);
        nh_.param<bool>("lio_builder/align_gravity", lio_params_.align_gravity, true);
        nh_.param<std::vector<double>>("lio_builder/imu_ext_rot", lio_params_.imu_ext_rot, std::vector<double>());
        nh_.param<std::vector<double>>("lio_builder/imu_ext_pos", lio_params_.imu_ext_pos, std::vector<double>());

        nh_.param<double>("localizer/refine_resolution", localizer_params_.refine_resolution, 0.2);
        nh_.param<double>("localizer/rough_resolution", localizer_params_.rough_resolution, 0.5);
        nh_.param<double>("localizer/refine_iter", localizer_params_.refine_iter, 5);
        nh_.param<double>("localizer/rough_iter", localizer_params_.rough_iter, 10);
        nh_.param<double>("localizer/thresh", localizer_params_.thresh, 0.15);

        nh_.param<double>("localizer/xy_offset", localizer_params_.xy_offset, 2.0);
        nh_.param<double>("localizer/yaw_resolution", localizer_params_.yaw_resolution, 0.5);
        nh_.param<int>("localizer/yaw_offset", localizer_params_.yaw_offset, 1);
    }

    void initSubscribers()
    {
        imu_sub_ = nh_.subscribe(imu_data_.topic, 1000, &ImuData::callback, &imu_data_);
        livox_sub_ = nh_.subscribe(livox_data_.topic, 1000, &LivoxData::callback, &livox_data_);
    }

    void initPublishers()
    {
        local_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("local_cloud", 1000);
        body_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("body_cloud", 1000);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("slam_odom", 1000);
        ori_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("ori_odom", 1000);
        odom_new_pub = nh_.advertise<nav_msgs::Odometry>("new_odom", 1000);
        Robot_base_odom_pub = nh_.advertise<nav_msgs::Odometry>("Robot_base_odom", 1000);
        map_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("map_cloud", 1000);
        Robot_base_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("Robot_base_cloud", 1000);
    }

    bool relocCallback(fastlio::SlamReLoc::Request &req, fastlio::SlamReLoc::Response &res)
    {
        std::cout << "relocCallback_node_setup" <<std::endl;
        std::string map_path = req.pcd_path;
        float x = req.x;
        float y = req.y;
        float z = req.z;
        float roll = req.roll;
        float pitch = req.pitch;
        float yaw = req.yaw;
        Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());
        Eigen::Quaternionf q = rollAngle * pitchAngle * yawAngle;
        {
            std::lock_guard<std::mutex> lock(shared_date_->service_mutex);
            shared_date_->halt_flag = false;
            shared_date_->service_called = true;
            shared_date_->localizer_activate = true;
            shared_date_->map_path = map_path;
            shared_date_->initial_guess.block<3, 3>(0, 0) = q.toRotationMatrix().cast<double>();
            shared_date_->initial_guess.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);
        }
        res.status = 1;
        res.message = "RELOCALIZE CALLED!";

        return true;
    }

    bool mapConvertCallback(fastlio::MapConvert::Request &req, fastlio::MapConvert::Response &res)
    {
        pcl::PCDReader reader;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        reader.read(req.map_path, *cloud);
        pcl::VoxelGrid<pcl::PointXYZI> down_sample_filter;
        down_sample_filter.setLeafSize(req.resolution, req.resolution, req.resolution);
        down_sample_filter.setInputCloud(cloud);
        down_sample_filter.filter(*cloud);

        fastlio::PointCloudXYZI::Ptr cloud_with_norm = fastlio::IcpLocalizer::addNorm(cloud);
        pcl::PCDWriter writer;
        writer.writeBinaryCompressed(req.save_path, *cloud_with_norm);
        res.message = "CONVERT SUCCESS!";
        res.status = 1;

        return true;
    }

    bool slamHoldCallback(fastlio::SlamHold::Request &req, fastlio::SlamHold::Response &res)
    {
        shared_date_->service_mutex.lock();
        shared_date_->halt_flag = true;
        shared_date_->reset_flag = true;
        shared_date_->service_mutex.unlock();
        res.message = "SLAM HALT!";
        res.status = 1;
        return true;
    }

    bool slamStartCallback(fastlio::SlamStart::Request &req, fastlio::SlamStart::Response &res)
    {
        shared_date_->service_mutex.lock();
        shared_date_->halt_flag = false;
        shared_date_->service_mutex.unlock();
        res.message = "SLAM START!";
        res.status = 1;
        return true;
    }

    bool slamRelocCheckCallback(fastlio::SlamRelocCheck::Request &req, fastlio::SlamRelocCheck::Response &res)
    {
        res.status = shared_date_->service_success;
        return true;
    }

    void initServices()
    {
        reloc_server_ = nh_.advertiseService("slam_reloc", &LocalizerROS::relocCallback, this);
        map_convert_server_ = nh_.advertiseService("map_convert", &LocalizerROS::mapConvertCallback, this);
        hold_server_ = nh_.advertiseService("slam_hold", &LocalizerROS::slamHoldCallback, this);
        start_server_ = nh_.advertiseService("slam_start", &LocalizerROS::slamStartCallback, this);
        reloc_check_server_ = nh_.advertiseService("slam_reloc_check", &LocalizerROS::slamRelocCheckCallback, this);
    }

    void publishCloud(ros::Publisher &publisher, const sensor_msgs::PointCloud2 &cloud_to_pub)
    {
        if (publisher.getNumSubscribers() == 0)
            return;
        publisher.publish(cloud_to_pub);
    }

    void publishCloud2(ros::Publisher &publisher, fastlio::PointCloudXYZI::Ptr inp, const double &timestamp)
    {
        if (publisher.getNumSubscribers() == 0)
            return;
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*inp, msg);
        if (timestamp < 0)
            msg.header.stamp = ros::Time().now();
        else
            msg.header.stamp = ros::Time().fromSec(timestamp);
        msg.header.frame_id = "world";
        publisher.publish(msg);
    }

    void publishOdom(const nav_msgs::Odometry &odom_to_pub)
    {

        // std::cout << "odom_pub_.getNumSubscribers()" << odom_pub_.getNumSubscribers() << std::endl;
        if (odom_new_pub.getNumSubscribers() == 0)
            return;
        odom_pub_.publish(odom_to_pub);

        nav_msgs::Odometry new_msg;

        new_msg = eigen2Odometry(new_mat_,
                                 current_state_.pos,
                                 local_frame_,
                                 body_frame_,
                                 current_time_);

        odom_new_pub.publish(new_msg);
    }


    void publishOdom2(const Eigen::Matrix3d &rot, const Eigen::Vector3d &pos,  const double &timestamp)
    {

        std::cout << "odom_pub_.getNumSubscribers()" << odom_pub_.getNumSubscribers() << std::endl;
        if (Robot_base_odom_pub.getNumSubscribers() == 0)
            return;
        // odom_pub_.publish(odom_to_pub);

        nav_msgs::Odometry odom;
        odom.header.frame_id = "world";
        odom.header.stamp = ros::Time().fromSec(timestamp);
        Eigen::Quaterniond q = Eigen::Quaterniond(rot);
        odom.pose.pose.position.x = pos(0);
        odom.pose.pose.position.y = pos(1);
        odom.pose.pose.position.z = pos(2);

        // odom.pose.pose.position.z = pos(2)+0.62;

        odom.pose.pose.orientation.w = q.w();
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();

        Robot_base_odom_pub.publish(odom);
    }

    void publishOdom3(const Eigen::Matrix3d &rot, const Eigen::Vector3d &pos,  const double &timestamp)
    {

        std::cout << "odom_pub_.getNumSubscribers()" << odom_pub_.getNumSubscribers() << std::endl;
        // if (odom_new_pub.getNumSubscribers() == 0)
        //     return;
        // odom_pub_.publish(odom_to_pub);

        nav_msgs::Odometry odom;
        odom.header.frame_id = "world";
        odom.header.stamp = ros::Time().fromSec(timestamp);
        Eigen::Quaterniond q = Eigen::Quaterniond(rot);
        odom.pose.pose.position.x = pos(0);
        odom.pose.pose.position.y = pos(1);
        odom.pose.pose.position.z = pos(2);

        odom.pose.pose.orientation.w = q.w();
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();

        ori_odom_pub_.publish(odom);
    }

    void systemReset()
    {
        offset_rot_ = Eigen::Matrix3d::Identity();
        offset_pos_ = Eigen::Vector3d::Zero();
        {
            std::lock_guard<std::mutex> lock(shared_date_->main_mutex);
            shared_date_->offset_rot = Eigen::Matrix3d::Identity();
            shared_date_->offset_pos = Eigen::Vector3d::Zero();
            shared_date_->service_success = false;
        }
        lio_builder_->reset();
    }

    void run()
    {
        while (ros::ok())
        {
            local_rate_->sleep();
            ros::spinOnce();
            if (terminate_flag)
                break;
            if (!measure_group_.syncPackage(imu_data_, livox_data_))
                continue;
            if (shared_date_->halt_flag)
                continue;

            if (shared_date_->reset_flag)
            {
                // ROS_INFO("SLAM RESET!");
                systemReset();
                shared_date_->service_mutex.lock();
                shared_date_->reset_flag = false;
                shared_date_->service_mutex.unlock();
            }

            lio_builder_->mapping(measure_group_);
            if (lio_builder_->currentStatus() == fastlio::Status::INITIALIZE)
                continue;
            current_time_ = measure_group_.lidar_time_end;
            current_state_ = lio_builder_->currentState();
            std::cout << "--------------------33333--------" << std::endl;
            current_cloud_body_ = lio_builder_->cloudUndistortedBody();
            {
                std::lock_guard<std::mutex> lock(shared_date_->main_mutex);
                shared_date_->local_rot = current_state_.rot.toRotationMatrix();
                shared_date_->local_pos = current_state_.pos;
                shared_date_->cloud = current_cloud_body_;
                offset_rot_ = shared_date_->offset_rot;
                offset_pos_ = shared_date_->offset_pos;
                global_change_ = shared_date_->global_change;
                std::cout << "--------------------global_change_:--------" << std::endl;
                std::cout << global_change_ << std::endl;

                shared_date_->pose_updated = true;
            }
            br_.sendTransform(eigen2Transform(
                current_state_.rot.toRotationMatrix(),
                current_state_.pos,
                local_frame_,
                body_frame_,
                current_time_));
            br_.sendTransform(eigen2Transform(
                offset_rot_,
                offset_pos_,
                global_frame_,
                local_frame_,
                current_time_));
            std::cout << "------------------111112222-" << std::endl;
            publishOdom(eigen2Odometry(current_state_.rot.toRotationMatrix(),
                                       current_state_.pos,
                                       local_frame_,
                                       body_frame_,
                                       current_time_));

            Eigen::Matrix3d body_to_local_rot = current_state_.rot.toRotationMatrix();
            Eigen::Vector3d body_to_local_pos = current_state_.pos;
            Eigen::Matrix3d local_to_global_rot = offset_rot_;
            Eigen::Vector3d local_to_global_pos = offset_pos_;

            Eigen::Matrix3d body_to_global_rot = local_to_global_rot * body_to_local_rot;
            Eigen::Vector3d body_to_global_pos = local_to_global_rot * body_to_local_pos + local_to_global_pos;

            


            // // 计算旋转角度（弧度）
            double angle1 = 55.0 / 180 * M_PI; // 绕x轴负方向旋转55度
            double angle2 = -0.0 / 180 * M_PI; // 绕z轴正方向旋转90度
            // 创建绕x轴负方向旋转55度的矩阵
            Eigen::Matrix3d mat_1_neg;
            // mat_1_neg << 1, 0, 0,
            //     0, cos(angle1), -sin(angle1),
            //     0, sin(angle1), cos(angle1);

            mat_1_neg << cos(angle1), 0, -sin(angle1),
                    0, 1, 0,
                    sin(angle1), 0, cos(angle1);

            // 创建绕z轴正方向旋转90度的矩阵
            Eigen::Matrix3d mat_z_pos;
            mat_z_pos << cos(angle2), -sin(angle2), 0,
                sin(angle2), cos(angle2), 0,
                0, 0, 1;

            Eigen::Matrix3d mat_z_clockwise;
            mat_z_clockwise << 0, -1, 0,
                            1, 0, 0,
                            0, 0, 1;

            // 应用逆变换到雷达的旋转和位置
            new_mat_ = current_state_.rot.toRotationMatrix() * mat_1_neg * mat_z_pos;


            Eigen::Matrix3d R_sensor2base = mat_1_neg* mat_z_pos;
            Eigen::Vector3d t_sensor2base(0.0, 0.0, 0.0);

            Eigen::Matrix4d T_sensor2base = Eigen::Matrix4d::Identity();
            T_sensor2base.block<3, 3>(0, 0) = R_sensor2base;
            T_sensor2base.block<3, 1>(0, 3) = t_sensor2base;

            // br_.sendTransform(eigen2Transform(R_sensor2base, t_sensor2base, body_frame_, Robot_base_frame_, current_time_));
            // publishOdom2(eigen2Odometry(R_sensor2base, t_sensor2base, body_frame_, Robot_base_frame_, current_time_));

            reloc_mat_rot = offset_rot_ * new_mat_;
            reloc_mat_pose = offset_rot_ * current_state_.pos + offset_pos_;

            publishOdom2(new_mat_,current_state_.pos, current_time_);
            // publishOdom2(current_state_.rot,current_state_.pos, current_time_);
            // publishOdom3(current_state_.rot.toRotationMatrix(),current_state_.pos, current_time_);


            publishCloud(body_cloud_pub_,
                         pcl2msg(current_cloud_body_,
                                 body_frame_,
                                 current_time_));

            fastlio::PointCloudXYZI::Ptr current_Robot_base_cloud_body_(new fastlio::PointCloudXYZI);
            
            current_Robot_base_cloud_body_ ->points.resize(current_cloud_body_->points.size());

            for(size_t i = 0; i< current_cloud_body_ ->points.size();++i){
                const auto& point = current_cloud_body_->points[i];
                Eigen::Vector3d p(point.x, point.y, point.z);
                // Eigen::Vector3d point_local = current_state_.rot.toRotationMatrix() * p + current_state_.pos;
                // Eigen::Vector3d transformed_p = offset_rot_ * p+offset_pos_;
                Eigen::Vector3d transformed_p = body_to_global_rot * p + body_to_global_pos;
                // transformed_p = mat_1_neg * mat_z_pos * transformed_p ;
                // Eigen::Vector3d transformed_p = mat_1_neg * mat_z_pos * p;
                current_Robot_base_cloud_body_ ->points[i].x = transformed_p.x();
                current_Robot_base_cloud_body_ ->points[i].y = transformed_p.y();
                current_Robot_base_cloud_body_ ->points[i].z = transformed_p.z();
                current_Robot_base_cloud_body_ ->points[i].intensity = point.intensity;
            }
            std::cout << "222-" << std::endl;
            
            // pcl::transformPointCloud(*current_cloud_body_, *current_Robot_base_cloud_body_, T_sensor2base.cast<float>());

            // publishCloud(Robot_base_cloud_pub_, pcl2msg(current_Robot_base_cloud_body_, Robot_base_frame_, current_time_));

            publishCloud2(Robot_base_cloud_pub_, current_Robot_base_cloud_body_, current_time_);

            // publishCloud(Robot_base_cloud_pub_, pcl2msg(current_Robot_base_cloud_body_, local_frame_, current_time_));
            
            publishCloud(local_cloud_pub_,
                         pcl2msg(lio_builder_->cloudWorld(),
                                 local_frame_,
                                 current_time_));
            
            if (publish_map_cloud_)
            {
                if (icp_localizer_->isInitialized())
                {
                    publishCloud(map_cloud_pub_,
                                 pcl2msg(icp_localizer_->getRoughMap(),
                                         global_frame_,
                                         current_time_));
                }
            }
        }

        localizer_thread_->join();
        std::cout << "LOCALIZER NODE IS DOWN!" << std::endl;
    }

private:
    ros::NodeHandle nh_;
    std::string body_frame_;
    std::string local_frame_;
    std::string global_frame_;
    std::string Robot_base_frame_;

    double current_time_;
    bool publish_map_cloud_;
    fastlio::state_ikfom current_state_;

    ImuData imu_data_;
    LivoxData livox_data_;
    MeasureGroup measure_group_;
    std::shared_ptr<SharedData> shared_date_;
    std::shared_ptr<ros::Rate> local_rate_;
    std::shared_ptr<ros::Rate> loop_rate_;
    tf2_ros::TransformBroadcaster &br_;
    fastlio::LioParams lio_params_;
    fastlio::LocalizerParams localizer_params_;
    std::shared_ptr<fastlio::LIOBuilder> lio_builder_;
    std::shared_ptr<fastlio::IcpLocalizer> icp_localizer_;
    LocalizerThread localizer_loop_;
    std::shared_ptr<std::thread> localizer_thread_;

    ros::Subscriber imu_sub_;

    ros::Subscriber livox_sub_;

    ros::Publisher odom_pub_;

    ros::Publisher odom_new_pub;

    ros::Publisher Robot_base_odom_pub;

    Eigen::Matrix3d new_mat_;
    Eigen::Matrix3d reloc_mat_rot = Eigen::Matrix3d::Identity();
    ;
    Eigen::Vector3d reloc_mat_pose = Eigen::Vector3d::Zero();

    ros::Publisher body_cloud_pub_;

    ros::Publisher local_cloud_pub_;

    ros::Publisher ori_odom_pub_;

    ros::Publisher Robot_base_cloud_pub_;

    ros::Publisher map_cloud_pub_;

    ros::ServiceServer reloc_server_;

    ros::ServiceServer map_convert_server_;

    ros::ServiceServer reloc_check_server_;

    ros::ServiceServer hold_server_;

    ros::ServiceServer start_server_;

    Eigen::Matrix3d offset_rot_ = Eigen::Matrix3d::Identity();

    Eigen::Vector3d offset_pos_ = Eigen::Vector3d::Zero();

    Eigen::Matrix4d global_change_;

    fastlio::PointCloudXYZI::Ptr current_cloud_body_;

    // fastlio::PointCloudXYZI::Ptr current_Robot_base_cloud_body_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localizer_node");
    tf2_ros::TransformBroadcaster br;
    signal(SIGINT, signalHandler);
    std::shared_ptr<SharedData> shared_date = std::make_shared<SharedData>();
    LocalizerROS localizer_ros(br, shared_date);
    localizer_ros.run();
    return 0;
}