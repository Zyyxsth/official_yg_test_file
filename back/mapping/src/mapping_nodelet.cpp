#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mapping/mapping.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <pcl_conversions/pcl_conversions.h>
#include <quadrotor_msgs/OccMap3d.h>
#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>

#include <atomic>
#include <thread>
// 添加必要的头文件
#include <visualization_msgs/Marker.h>

namespace mapping {

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry>
    ImageOdomSyncPolicy;
typedef message_filters::Synchronizer<ImageOdomSyncPolicy>
    ImageOdomSynchronizer;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> LocalmapOdomSyncPolicy;
typedef message_filters::Synchronizer<LocalmapOdomSyncPolicy> LocalmapOdomSynchronizer;

struct CamConfig {
  // camera paramters
  double rate;
  double range;
  int width;
  int height;
  double fx;
  double fy;
  double cx;
  double cy;
  double depth_scaling_factor;
};

class Nodelet : public nodelet::Nodelet {
 private:
  std::thread initThread_;
  CamConfig camConfig_;  // just store the parameters of camera

  int down_sample_factor_;

  Eigen::Matrix3d cam2body_R_;
  Eigen::Vector3d cam2body_p_;

  // just for depth filter
  Eigen::Vector3d last_cam_p_;
  Eigen::Quaterniond last_cam_q_;
  bool get_first_frame_ = false;
  cv::Mat last_depth_;
  double depth_filter_tolerance_;
  double depth_filter_mindist_;
  int depth_filter_margin_;

  std::atomic_flag callback_lock_ = ATOMIC_FLAG_INIT;
  message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> local_map_sub_;
  std::shared_ptr<ImageOdomSynchronizer> depth_odom_sync_Ptr_;
  std::shared_ptr<LocalmapOdomSynchronizer> local_map_odom_sync_Ptr_;
  ros::Publisher gridmap_inflate_pub_, local_pc_pub_, pcl_pub_;
  // shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom2_sub_;
  ros::Subscriber indep_cloud_sub_,indep_odom_sub_;
  ros::Publisher gridmap_inflate_vs_pub;
  double local_margin;
  ros::Publisher area_visualization_pub_; 
  // 存放全局地图加载时的障碍格子索引
  std::vector<Eigen::Vector3i> init_obstacles_;

  // NOTE just for global map in simulation
  ros::Timer global_map_timer_;
  ros::Subscriber map_pc_sub_;
  bool map_recieved_ = false;
  bool use_global_map_ = false;
  // ros::Publisher gridmap_vs_pub, gridmap_inflate_vs_pub;
  bool remove_floor_ceil_ = false;

  // NOTE for mask target
  bool use_mask_ = false;
  ros::Subscriber target_odom_sub_;
  std::atomic_flag target_lock_ = ATOMIC_FLAG_INIT;
  Eigen::Vector3d target_odom_;
  ros::Subscriber gridmap_inflate_sub;

  OccGridMap gridmap_;
  int inflate_size_;
  Eigen::Vector3d body_pos;
  Eigen::Quaterniond body_qua;
  Eigen::Vector3d cam_pos;

  quadrotor_msgs::OccMap3d gridmap_msg;
  


  void depth_odom_callback(const sensor_msgs::ImageConstPtr& depth_msg,
                           const nav_msgs::OdometryConstPtr& odom_msg) {
    if (callback_lock_.test_and_set()) {
      return;
    }
    ros::Time t1, t2;
    // t1 = ros::Time::now();
    Eigen::Vector3d body_p(odom_msg->pose.pose.position.x,
                           odom_msg->pose.pose.position.y,
                           odom_msg->pose.pose.position.z);
    Eigen::Quaterniond body_q(
        odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);
    Eigen::Vector3d cam_p = body_q.toRotationMatrix() * cam2body_p_ + body_p;
    Eigen::Quaterniond cam_q = body_q * Eigen::Quaterniond(cam2body_R_);
    cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(depth_msg);
    if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      (depth_ptr->image).convertTo(depth_ptr->image, CV_16UC1, camConfig_.depth_scaling_factor);
    }
    cv::Mat depth_img = depth_ptr->image;

    // pub target

    int nr = depth_img.rows;
    int nc = depth_img.cols;
    std::vector<Eigen::Vector3d> obs_pts;
    // put the points of the depth into the list of obs_points

    // TODO depth filter

    // t1 = ros::Time::now();
    for (int i = depth_filter_margin_; i < nr - depth_filter_margin_; i += down_sample_factor_) {
      for (int j = depth_filter_margin_; j < nc - depth_filter_margin_; j += down_sample_factor_) {
        // (x,y,z) in camera frame
        double z = (depth_img.at<uint16_t>(i, j)) / camConfig_.depth_scaling_factor;
        if (depth_img.at<uint16_t>(i, j) == 0) {
          z = camConfig_.range + 0.5;
        }
        if (std::isnan(z) || std::isinf(z))
          continue;
        if (z < depth_filter_mindist_) {
          continue;
        }
        double y = (i - camConfig_.cy) * z / camConfig_.fy;
        double x = (j - camConfig_.cx) * z / camConfig_.fx;
        Eigen::Vector3d p(x, y, z);
        p = cam_q * p + cam_p;
        bool good_point = true;
        if (get_first_frame_) {
          // NOTE depth filter:
          Eigen::Vector3d p_rev_proj =
              last_cam_q_.inverse().toRotationMatrix() * (p - last_cam_p_);
          double vv = p_rev_proj.y() * camConfig_.fy / p_rev_proj.z() + camConfig_.cy;
          double uu = p_rev_proj.x() * camConfig_.fx / p_rev_proj.z() + camConfig_.cx;
          if (vv >= 0 && vv < nr && uu >= 0 && uu < nc) {
            double drift_dis = fabs(last_depth_.at<uint16_t>((int)vv, (int)uu) / camConfig_.depth_scaling_factor - p_rev_proj.z());
            if (drift_dis > depth_filter_tolerance_) {
              good_point = false;
            }
          }
        }
        if (good_point) {
          obs_pts.push_back(p);
        }
      }
    }
    last_depth_ = depth_img;
    last_cam_p_ = cam_p;
    last_cam_q_ = cam_q;
    get_first_frame_ = true;
    gridmap_.updateMap(cam_p, obs_pts);

    // NOTE use mask
    if (use_mask_) {  // mask target
      while (target_lock_.test_and_set())
        ;
      Eigen::Vector3d ld = target_odom_;
      Eigen::Vector3d ru = target_odom_;
      ld.x() -= 0.5;
      ld.y() -= 0.5;
      ld.z() -= 1.0;
      ru.x() += 0.5;
      ru.y() += 0.5;
      ru.z() += 1.0;
      gridmap_.setFree(ld, ru);
      target_lock_.clear();
    }


    quadrotor_msgs::OccMap3d gridmap_msg;
    gridmap_msg.header.frame_id = "world";
    gridmap_msg.header.stamp = ros::Time::now();
    gridmap_.to_msg(gridmap_msg);
    gridmap_inflate_pub_.publish(gridmap_msg);

    callback_lock_.clear();
  }


  void odomCallback(const nav_msgs::OdometryConstPtr &odom)
  {
    /* get pose */
    body_pos = Eigen::Vector3d(odom->pose.pose.position.x,
      odom->pose.pose.position.y,
      odom->pose.pose.position.z);

    body_qua = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                 odom->pose.pose.orientation.x,
                                                 odom->pose.pose.orientation.y,
                                                 odom->pose.pose.orientation.z);
    cam_pos = body_pos;

    // std::cout << body_pos << std::endl;
  }

  void updateLocalMap(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    if (callback_lock_.test_and_set()) {
      std::cout << "i don't know the meaning of test_and_set()" <<std::endl;
      return;
    }

    // // 1. 计算原始的min_id和max_id（基于cam_pos和local_margin）
    // Eigen::Vector3i min_id_raw = gridmap_.pos2idx(cam_pos + Eigen::Vector3d(0.5*local_margin, -local_margin, -0.5*local_margin));
    // Eigen::Vector3i max_id_raw = gridmap_.pos2idx(cam_pos + Eigen::Vector3d(1.0*local_margin, local_margin, local_margin));

    // // 2. 确定地图的有效索引范围
    // Eigen::Vector3i map_min_idx(gridmap_.offset_x, gridmap_.offset_y, gridmap_.offset_z);
    // Eigen::Vector3i map_max_idx(
    //   gridmap_.offset_x + gridmap_.size_x - 1,
    //   gridmap_.offset_y + gridmap_.size_y - 1,
    //   gridmap_.offset_z + gridmap_.size_z - 1
    // );

    // // 3. 裁剪min_id和max_id到有效范围内（确保不超出地图边界）
    // Eigen::Vector3i min_id = min_id_raw.cwiseMax(map_min_idx);  // 取较大值，避免小于地图最小索引
    // Eigen::Vector3i max_id = max_id_raw.cwiseMin(map_max_idx);  // 取较小值，避免大于地图最大索引

    // // 4. 额外检查：若裁剪后min_id > max_id，说明范围无效，直接跳过
    // if (min_id.x() > max_id.x() || min_id.y() > max_id.y() || min_id.z() > max_id.z()) {
    //   ROS_WARN("Local map range is invalid after clipping.");
    //   return;
    // }
    // //i guess is the question that make the map yi chu ?

    // publishAreaMarker(min_id, max_id);


    // // 清除局部地图信息
    // for (int x = min_id.x(); x <= max_id.x(); ++x) {
    //     for (int y = min_id.y(); y <= max_id.y(); ++y) {
    //         for (int z = min_id.z(); z <= max_id.z(); ++z) {
    //             Eigen::Vector3i id(x, y, z);
    //             if (gridmap_.isInMap(id)) {
    //                 // std::cout << "2 big lei" << std::endl;
    //                 gridmap_.setFree2(id);
    //             }
    //         }
    //     }
    // }

    // // std::cout << "step2 wrong?" << std::endl;

    // // 2. 将点云转换为栅格索引，并标记到临时数组中
    // pcl::PointCloud<pcl::PointXYZ> point_cloud;
    // pcl::fromROSMsg(*cloud_msg, point_cloud);

    // // 创建临时数组，记录更新区域内哪些栅格被点云覆盖
    // Eigen::Vector3i region_size = max_id - min_id + Eigen::Vector3i::Ones();
    // std::vector<std::vector<std::vector<bool>>> has_point(
    //   region_size.x(), 
    //   std::vector<std::vector<bool>>(region_size.y(), 
    //   std::vector<bool>(region_size.z(), false))
    // );

    // // std::cout << "step3 wrong?" << std::endl;

    // // 遍历点云，标记覆盖的栅格
    // for (const auto& pt : point_cloud) {
    //   if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) continue;
    //   Eigen::Vector3d p(pt.x, pt.y, pt.z);
    //   Eigen::Vector3i idx = gridmap_.pos2idx(p);

    //   // 只处理更新区域内的点云
    //   if (idx.x() >= min_id.x() && idx.x() <= max_id.x() &&
    //       idx.y() >= min_id.y() && idx.y() <= max_id.y() &&
    //       idx.z() >= min_id.z() && idx.z() <= max_id.z() &&
    //       gridmap_.isInMap(idx)) {
    //       Eigen::Vector3i rel_idx = idx - min_id;
    //       has_point[rel_idx.x()][rel_idx.y()][rel_idx.z()] = true;
    //   }
    // }

    // // std::cout << "step4 wrong?" << std::endl;

    // // 3. 遍历更新区域：有点云→hit，无点云→mis（因明确是观测区域）
    // for (int x = 0; x < region_size.x(); ++x) {
    //   for (int y = 0; y < region_size.y(); ++y) {
    //     for (int z = 0; z < region_size.z(); ++z) {
    //       Eigen::Vector3i global_id(
    //         min_id.x() + x,
    //         min_id.y() + y,
    //         min_id.z() + z
    //       );
    //       if (!gridmap_.isInMap(global_id)) continue;

    //       if (has_point[x][y][z]) {
    //         // std::cout << "step4.5 wrong?" << std::endl;
    //         // std::cout << "enter here" << std::endl;
    //         gridmap_.testhit(global_id);  // 有点云→占据
    //         // std::cout << "step4.8 wrong?" << std::endl;
    //       } else {
    //         // std::cout << "step4.6 wrong?" << std::endl;
    //         gridmap_.testmis(global_id);  // 无点云→自由（因明确是观测区域）
    //         // std::cout << "step4.7 wrong?" << std::endl;
    //       }
    //     }
    //   }
    // }

    // std::cout << "step5 wrong?" << std::endl;



    // // 处理雷达感知到的局部点云信息
    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    pcl::fromROSMsg(*cloud_msg, point_cloud);
    std::vector<Eigen::Vector3d> obs_pts;
    for(const auto& pt : point_cloud){
      Eigen::Vector3d p(pt.x, pt.y, pt.z);
      obs_pts.push_back(p);
    }
    gridmap_.updateMap(cam_pos, obs_pts);

    if (!init_obstacles_.empty()) {
      int still_occ = 0, cleared = 0;
      for (const auto& idx : init_obstacles_) {
        if (gridmap_.isOccupied(idx)) {
          still_occ++;
        } else {
          cleared++;
        }
      }
      ROS_INFO("[InitObstacleCheck] still_occ=%d cleared=%d", still_occ, cleared);
    }
    // for (const auto& pt : point_cloud) {
    //   Eigen::Vector3d p(pt.x, pt.y, pt.z);
    //   if(gridmap_.isInMap(p)){

        






    //     // std::cout << "3 big lei" << std::endl;
    //     gridmap_.setOcc(p);
        
    //   }
    //   // if(pt.z>0.1 && pt.z < 1.2){
    //   //   for (int i=1 ; i<51 ; i++){
    //   //     Eigen::Vector3d p_ugv(pt.x, pt.y, pt.z+(0.15*i));
    //   //     gridmap_.setOcc(p_ugv);
    //   //   }
        
    //   // }
    // }

    // std::cout << "1 big lei" << std::endl;


    //-----------bu zhi dao zhe li dui bu dui
    gridmap_msg.header.frame_id = "world";
    gridmap_msg.header.stamp = ros::Time::now();
    gridmap_.to_msg(gridmap_msg);
    gridmap_inflate_pub_.publish(gridmap_msg);

    callback_lock_.clear();

}

  // NOTE
  void target_odom_callback(const nav_msgs::OdometryConstPtr& msgPtr) {
    while (target_lock_.test_and_set())
      ;
    target_odom_.x() = msgPtr->pose.pose.position.x;
    target_odom_.y() = msgPtr->pose.pose.position.y;
    target_odom_.z() = msgPtr->pose.pose.position.z;
    target_lock_.clear();
  }

  // 在计算出min_id和max_id后调用此函数
void publishAreaMarker(const Eigen::Vector3i& min_id, const Eigen::Vector3i& max_id) {
  // 创建Marker消息
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";  // 与地图坐标系一致
  marker.header.stamp = ros::Time::now();
  marker.ns = "local_map_area";  // 命名空间，避免与其他Marker冲突
  marker.id = 0;  // 同一命名空间下唯一ID
  marker.type = visualization_msgs::Marker::CUBE;  // 类型为立方体
  marker.action = visualization_msgs::Marker::ADD;  // 动作：添加

  // 1. 将栅格索引转换为世界坐标（取栅格的边界，而非中心点）
  // 注意：idx2pos返回的是栅格中心点，需计算区域的实际边界
  Eigen::Vector3d min_pos = gridmap_.idx2pos(min_id) - Eigen::Vector3d(gridmap_.resolution/2, gridmap_.resolution/2, gridmap_.resolution/2);
  Eigen::Vector3d max_pos = gridmap_.idx2pos(max_id) + Eigen::Vector3d(gridmap_.resolution/2, gridmap_.resolution/2, gridmap_.resolution/2);

  // 2. 计算立方体的中心点和尺寸
  Eigen::Vector3d center = (min_pos + max_pos) / 2.0;  // 中心点坐标
  Eigen::Vector3d size = max_pos - min_pos;  // 长宽高（世界坐标单位：米）

  // 3. 设置Marker位置和尺寸
  marker.pose.position.x = center.x();
  marker.pose.position.y = center.y();
  marker.pose.position.z = center.z();
  marker.pose.orientation.w = 1.0;  // 无旋转

  marker.scale.x = size.x();  // x方向长度
  marker.scale.y = size.y();  // y方向长度
  marker.scale.z = size.z();  // z方向长度

  // 4. 设置颜色（半透明蓝色，便于观察）
  marker.color.r = 0.0;    // 红色分量
  marker.color.g = 0.0;    // 绿色分量
  marker.color.b = 1.0;    // 蓝色分量
  marker.color.a = 0.5;    // 透明度（0.5表示半透明）

  // 5. 设置生命周期（0表示永久存在，直到被覆盖）
  marker.lifetime = ros::Duration(0);

  // 6. 发布Marker
  area_visualization_pub_.publish(marker);
}

  // NOTE just for global map in simulation
  void map_call_back(const sensor_msgs::PointCloud2ConstPtr& msgPtr) {
    if (map_recieved_) {
      return;
    }
    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    pcl::fromROSMsg(*msgPtr, point_cloud);
    for (const auto& pt : point_cloud) {
      Eigen::Vector3d p(pt.x, pt.y, pt.z);
      gridmap_.setOcc(p);
      //------------------------------------------------------------------------------------------所有障碍物加高了4.5m(4.5/0.15+1=31)吧------------------------------------------
      // 12.19 
      if(pt.z>0.1 && pt.z < 1.2){
        for (int i=1 ; i<31 ; i++){
          Eigen::Vector3d p_ugv(pt.x, pt.y, pt.z+(0.15*i));
          gridmap_.setOcc(p_ugv);
        }
        
      }
      
  
     
      //--------------------------------------------------------------------------------------------------------------------------------

    }
    gridmap_.inflate(inflate_size_);
    ROS_WARN("[mapping] GLOBAL MAP REVIEVED!");
    map_recieved_ = true;
    return;
  }
  void global_map_timer_callback(const ros::TimerEvent& event) {
    if (!map_recieved_) {
      std::cout << "map_received" << std::endl;
      return;
    }
    quadrotor_msgs::OccMap3d gridmap_msg;
    gridmap_.to_msg(gridmap_msg);
    gridmap_inflate_pub_.publish(gridmap_msg);
    std::cout << "publish global map timer callback" <<std::endl;
  }

  void once_global_map_call_back(const sensor_msgs::PointCloud2ConstPtr& msgPtr){
    if (map_recieved_) {
      return;
    }
    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    pcl::fromROSMsg(*msgPtr, point_cloud);
    for (const auto& pt : point_cloud) {
      Eigen::Vector3d p(pt.x, pt.y, pt.z);
      gridmap_.setOcc(p);
      //------------------------------------------------------------------------------------------所有障碍物加高了4.5m(4.5/0.15+1=31)吧------------------------------------------
      // 12.19 
      // if(pt.z>0.1 && pt.z < 1.2){
      //   for (int i=1 ; i<50 ; i++){
      //     Eigen::Vector3d p_ugv(pt.x, pt.y, pt.z+(0.15*i));
      //     gridmap_.setOcc(p_ugv);
      //   }
        
      // }
      
  
     
      //--------------------------------------------------------------------------------------------------------------------------------

    }
    gridmap_.inflate(inflate_size_);
    ROS_WARN("[mapping] GLOBAL MAP REVIEVED!");
    map_recieved_ = true;
    // quadrotor_msgs::OccMap3d gridmap_msg;
    gridmap_msg.header.frame_id = "world";
    gridmap_msg.header.stamp = ros::Time::now();
    gridmap_.to_msg(gridmap_msg);
    gridmap_inflate_pub_.publish(gridmap_msg);
    std::cout << "publish global map timer callback" <<std::endl;
    return;
  }


  void once_global_map_call_back2(const sensor_msgs::PointCloud2ConstPtr& msgPtr){
    // if (map_recieved_) {
    //   return;
    // }
    // pcl::PointCloud<pcl::PointXYZ> point_cloud;
    // pcl::fromROSMsg(*msgPtr, point_cloud);
  
    // for (const auto& pt : point_cloud) {
    //   Eigen::Vector3d p(pt.x, pt.y, pt.z);
    //   // gridmap_.setOccGlobal(p);   // ✅ 通过新接口走完整流程
    //   gridmap_.addGlobalAsHit(p, -1, false);

    //   // // 记录格子索引，方便后续追踪
    //   init_obstacles_.push_back(gridmap_.pos2idx(p));
    // }
  
    // // 膨胀处理
    // gridmap_.inflate(inflate_size_);
  
    // ROS_WARN("[mapping] GLOBAL MAP REVIEVED!");
    // ROS_WARN("once_global_map_call_back triggered at time %.2f", ros::Time::now().toSec());
    // map_recieved_ = true;
  
    // // 发布一次全局地图
    // gridmap_msg.header.frame_id = "world";
    // gridmap_msg.header.stamp = ros::Time::now();
    // gridmap_.to_msg(gridmap_msg);
    // gridmap_inflate_pub_.publish(gridmap_msg);
  
    // return;
    if (map_recieved_) return;

    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    pcl::fromROSMsg(*msgPtr, point_cloud);

    // 传感器原点（假设在世界原点）
    Eigen::Vector3d sensor_origin(0.0, 0.0, 0.0);

    for (const auto& pt : point_cloud) {
        Eigen::Vector3d p(pt.x, pt.y, pt.z);

        // 使用全局点作为一次观测 (hit + raycasting mis)
        gridmap_.addGlobalAsObservation(sensor_origin, p, -1); 
        // 这里 10 是权重，可以调大或调小
    }

    gridmap_.inflate(inflate_size_);
    ROS_WARN("[mapping] GLOBAL MAP RECEIVED!");
    map_recieved_ = true;

    // 发布一次全局地图
    gridmap_msg.header.frame_id = "world";
    gridmap_msg.header.stamp = ros::Time::now();
    gridmap_.to_msg(gridmap_msg);
    gridmap_inflate_pub_.publish(gridmap_msg);

    std::cout << "publish global map (with observation)" << std::endl;





  }
  

  

  void gridmap_inflate_vs_callback(const quadrotor_msgs::OccMap3dConstPtr& msgPtr) {
    // std::cout << "xxxxxxxxxxxxxxxxxxxx" <<std::endl;
    mapping::OccGridMap gridmap_vs;
    gridmap_vs.from_msg(*msgPtr);
    sensor_msgs::PointCloud2 pc;
    if (remove_floor_ceil_) {
      gridmap_vs.occ2pc(pc, 0.5, 2.5);
    } else {
      gridmap_vs.occ2pc(pc, -3.0, 1.0);
    }


    pc.header.frame_id = "world";
    gridmap_inflate_vs_pub.publish(pc);
    // std::cout << "xaaaaaaaaaaaaaa" <<std::endl;
  }


  void init(ros::NodeHandle& nh) {

    std::vector<double> tmp;
    if (nh.param<std::vector<double>>("cam2body_R", tmp, std::vector<double>())) {
      cam2body_R_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(tmp.data(), 3, 3);
    }
    if (nh.param<std::vector<double>>("cam2body_p", tmp, std::vector<double>())) {
      cam2body_p_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(tmp.data(), 3, 1);
    }

    double res;
    Eigen::Vector3d map_size;
    // NOTE whether to use global map
    nh.getParam("use_global_map", use_global_map_);
    if (use_global_map_) {
      double x, y, z, res;
      nh.getParam("x_length", x);
      nh.getParam("y_length", y);
      nh.getParam("z_length", z);
      nh.getParam("resolution", res);
      nh.getParam("inflate_size", inflate_size_);
      gridmap_.setup(res, Eigen::Vector3d(x, y, z), 10, true);
      nh.getParam("local_margin", local_margin);
      int p_min, p_max, p_hit, p_mis, p_occ, p_def;
      nh.getParam("p_min", p_min);
      nh.getParam("p_max", p_max);
      nh.getParam("p_hit", p_hit);
      nh.getParam("p_mis", p_mis);
      nh.getParam("p_occ", p_occ);
      nh.getParam("p_def", p_def);
      gridmap_.setupP(p_min, p_max, p_hit, p_mis, p_occ, p_def);
      std::cout << "获取到的参数：x=" << x << ", y=" << y << ", z=" << z << ", res=" << res << std::endl;
      ROS_INFO_STREAM("Grid map offset = ("
        << gridmap_.offset_x << ", "
        << gridmap_.offset_y << ", "
        << gridmap_.offset_z << ")"
        << " , size = (" << gridmap_.size_x
        << ", " << gridmap_.size_y
        << ", " << gridmap_.size_z << ")"
        << " , resolution = " << gridmap_.resolution);

    } else {
      // camera parameters
      nh.getParam("camera_rate", camConfig_.rate);
      nh.getParam("camera_range", camConfig_.range);
      nh.getParam("cam_width", camConfig_.width);
      nh.getParam("cam_height", camConfig_.height);
      nh.getParam("cam_fx", camConfig_.fx);
      nh.getParam("cam_fy", camConfig_.fy);
      nh.getParam("cam_cx", camConfig_.cx);
      nh.getParam("cam_cy", camConfig_.cy);
      nh.getParam("depth_scaling_factor", camConfig_.depth_scaling_factor);
      // mapping parameters
      nh.getParam("down_sample_factor", down_sample_factor_);
      nh.getParam("resolution", res);
      nh.getParam("local_x", map_size.x());
      nh.getParam("local_y", map_size.y());
      nh.getParam("local_z", map_size.z());
      nh.getParam("inflate_size", inflate_size_);
      gridmap_.setup(res, map_size, camConfig_.range);
      // depth filter parameters
      nh.getParam("depth_filter_tolerance", depth_filter_tolerance_);
      nh.getParam("depth_filter_mindist", depth_filter_mindist_);
      nh.getParam("depth_filter_margin", depth_filter_margin_);
      nh.getParam("local_margin", local_margin);
      // raycasting parameters
      int p_min, p_max, p_hit, p_mis, p_occ, p_def;
      nh.getParam("p_min", p_min);
      nh.getParam("p_max", p_max);
      nh.getParam("p_hit", p_hit);
      nh.getParam("p_mis", p_mis);
      nh.getParam("p_occ", p_occ);
      nh.getParam("p_def", p_def);
      gridmap_.setupP(p_min, p_max, p_hit, p_mis, p_occ, p_def);
    }
    gridmap_.inflate_size = inflate_size_;
    // use mask parameter
    nh.getParam("use_mask", use_mask_);

    gridmap_inflate_pub_ = nh.advertise<quadrotor_msgs::OccMap3d>("gridmap_inflate", 10);
    gridmap_inflate_vs_pub = nh.advertise<sensor_msgs::PointCloud2>("vs_gridmap_inflate", 10);
    

    if (use_global_map_) {
      // map_pc_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("global_map", 1, &Nodelet::map_call_back, this);
      global_map_timer_ = nh.createTimer(ros::Duration(20.0), &Nodelet::global_map_timer_callback, this);
      map_pc_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("global_map", 1, &Nodelet::once_global_map_call_back2, this);
      indep_odom_sub_ = nh.subscribe<nav_msgs::Odometry>("odom", 10, &Nodelet::odomCallback, this);
      indep_cloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("aaa_local_map", 10, &Nodelet::updateLocalMap, this);
      gridmap_inflate_sub = nh.subscribe<quadrotor_msgs::OccMap3d>("gridmap_inflate", 10, &Nodelet::gridmap_inflate_vs_callback, this);
      area_visualization_pub_ = nh.advertise<visualization_msgs::Marker>("local_map_area_marker", 1);
    } else {
      //----------for update local map by using camera-------------
      depth_sub_.subscribe(nh, "depth", 1);
      odom_sub_.subscribe(nh, "odom", 50);
      depth_odom_sync_Ptr_ = std::make_shared<ImageOdomSynchronizer>(ImageOdomSyncPolicy(100), depth_sub_, odom_sub_);
      depth_odom_sync_Ptr_->registerCallback(boost::bind(&Nodelet::depth_odom_callback, this, _1, _2));
    }

    if (use_mask_) {
      // target_odom_sub_ = nh.subscribe<nav_msgs::Odometry>("target", 1, &Nodelet::target_odom_callback, this, ros::TransportHints().tcpNoDelay());

    }
  }

 public:
  void onInit(void) {
    ros::NodeHandle nh(getMTPrivateNodeHandle());
    initThread_ = std::thread(std::bind(&Nodelet::init, this, nh));
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace mapping

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mapping::Nodelet, nodelet::Nodelet);