#include <SIYIClient/SIYIClient.h>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/Bool.h"
#include <mapping/mapping.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <quadrotor_msgs/OccMap3d.h>
#include <quadrotor_msgs/PolyTraj.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/ReplanState.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <traj_opt/traj_opt.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Core>
#include <atomic>
#include <cmath>
#include <env/env.hpp>
#include <prediction/prediction.hpp>
#include <thread>
#include <vector>
#include <visualization/visualization.hpp>
#include <wr_msg/wr_msg.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <mpc/Polynome.h>
#include <unistd.h>
#include <nlohmann/json.hpp>
#include <motion_msgs/Diablo_Ctrl.h>
// #include "diablo_sdk/Diablo_Ctrl.h"

/*
轮腿更改：
1. hover取消，更改至at_goal(pub_hover_p全部注释掉了)
2. land_triger取消

*/

namespace planning {

Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");

// 11.11
enum FSM_EXEC_STATE {
  INIT,
  REC_MAP,
  CHANGE_TARGET,
  GEN_NEW_TRAJ,
  REPLAN_TRAJ,
  EXEC_TRAJ,
  ROTATE,
  DETECT,
  STOP,
  EMERGE_STOP
};

class Nodelet : public nodelet::Nodelet {
 private:
  std::thread initThread_;
  ros::Subscriber gridmap_sub_, odom_sub_, target_sub_, triger_sub_, land_triger_sub_, wheel_leg_pos_cmd_subscriber;
  ros::Timer plan_timer_;
  ros::Timer exec_timer_;
  // 0.1.1版本增加重规划逻辑
  ros::Publisher traj_pub_, heartbeat_pub_, replanState_pub_, pos_cmd_pub_, stop_pub,triger_pub;
  std_msgs::Bool stop_msg;
  ros::Publisher traj_vis_pub;  // 发布traj的轨迹可视化
  ros::Publisher traj_vis_pub2, rotation_cmd_pub_;
  ros::Publisher last_traj_t_rest_pub_;
  double last_traj_t_rest_global_;

  std::shared_ptr<mapping::OccGridMap> gridmapPtr_;
  std::shared_ptr<env::Env> envPtr_;
  std::shared_ptr<visualization::Visualization> visPtr_;
  std::shared_ptr<traj_opt::TrajOpt> trajOptPtr_;
  std::shared_ptr<prediction::Predict> prePtr_;

  // NOTE planning or fake target
  bool fake_ = false;
  Eigen::Vector3d goal_;
  Eigen::Vector3d land_p_;
  Eigen::Vector3d local_goal;      // 当前的规划目标
  Eigen::Vector3d odom_p, odom_v;  // 当前的位姿信息
  double odom_yaw_rad;
  Eigen::Vector3d temp_goal_point;  // 当前的goal是哪一个
  double yaw_goal_temp;
  Trajectory traj_last;
  ros::Time last_start_stamp;
  double last_yaw_ = 0;
  Eigen::Quaterniond land_q_;
  bool detect_sim = false;
  Eigen::Vector4d preset_goal_quat;
  Eigen::Vector4d now_quat;
  // 预设的odom信息
  nav_msgs::Odometry preset_odom;
  // 当前的odom信息
  nav_msgs::Odometry current_odom;
  double virtual_ceiling_height_;

  // NOTE just for debug
  bool debug_ = false;
  quadrotor_msgs::ReplanState replanStateMsg_;
  ros::Publisher gridmap_pub_, inflate_gridmap_pub_;
  quadrotor_msgs::OccMap3d occmap_msg_;
  ros::Publisher mpc_traj_pub_;
  ros::Publisher mpc_vis_pub;

  double tracking_dur_, tracking_dist_, tolerance_d_;

  Trajectory traj_poly_;
  ros::Time replan_stamp_;
  int traj_id_ = 0;
  bool wait_hover_ = true;
  bool emg_stop = true;

  nav_msgs::Odometry odom_msg_, target_msg_;
  quadrotor_msgs::OccMap3d map_msg_;

  // 9.9.11 try-------------------stop--------------------
  bool ifstop = false;
  // 9.9.11 try-------------------stop--------------------

  std::atomic_flag odom_lock_ = ATOMIC_FLAG_INIT;
  std::atomic_flag target_lock_ = ATOMIC_FLAG_INIT;
  std::atomic_flag gridmap_lock_ = ATOMIC_FLAG_INIT;
  std::atomic_bool odom_received_ = ATOMIC_VAR_INIT(false);
  std::atomic_bool map_received_ = ATOMIC_VAR_INIT(false);
  std::atomic_bool triger_received_ = ATOMIC_VAR_INIT(false);
  std::atomic_bool target_received_ = ATOMIC_VAR_INIT(false);
  std::atomic_bool land_triger_received_ = ATOMIC_VAR_INIT(false);

  // 11.11
  FSM_EXEC_STATE exec_state_;

  // 12.2
  std::queue<Eigen::Vector3d> pos_queue;
  std::queue<Eigen::Vector2d> siyi_queue; 
  std::queue<Eigen::Vector4d> quat_queue;// 储存目标的四元数
  std::queue<double> time_queue;
  double siyi1, siyi2;
  int current_file_name;
  double time_q1;
  int goal_num = 0;  //看有多少个goal
  int temp_goal;

  // 角度阈值
  const double ANGLE_THRESHOLD = 7.0; // 单位：度

  const double ANGULAR_VELOCITY_VALUE = 0.5;

  // siyi 11.26
  SIYIClient siyiClient;
  std::string rtsp_url = "rtsp://192.168.1.25:8554/main.264";
  int detect_num = 0;
  std::vector<cv::Vec2f> detect_dir;

  // 12.9 vision
  bool vision_rotate = false;

  bool if_end = false;

  void pub_hover_p(const Eigen::Vector3d& hover_p, const ros::Time& stamp) {
    quadrotor_msgs::PolyTraj traj_msg;
    traj_msg.hover = true;
    traj_msg.hover_p.resize(3);
    for (int i = 0; i < 3; ++i) {
      traj_msg.hover_p[i] = hover_p[i];
    }
    traj_msg.start_time = stamp;
    traj_msg.traj_id = traj_id_++;
    traj_pub_.publish(traj_msg);
  }

  void renderTraj(Trajectory traj) {
    int id = 4;
    visualization_msgs::Marker traj_vis;
    traj_vis.header.stamp = ros::Time::now();
    traj_vis.header.frame_id = "world";
    traj_vis.id = id++;
    traj_vis.type = visualization_msgs::Marker::LINE_STRIP;
    traj_vis.scale.x = 0.2;
    traj_vis.scale.y = 0.2;
    traj_vis.scale.z = 0.2;
    traj_vis.pose.orientation.x = 0.0;
    traj_vis.pose.orientation.y = 0.0;
    traj_vis.pose.orientation.z = 0.0;
    traj_vis.pose.orientation.w = 1.0;

    traj_vis.color.a = 1.0;
    traj_vis.color.r = 1.0;
    traj_vis.color.g = 1.0;
    traj_vis.color.b = 0.5;
    geometry_msgs::Point pt, pt_last, pt_des, pt_des_last;
    Eigen::Vector3d pos;

    double t_duration = traj.getTotalDuration();
    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<double> zs;
    double last_zs;

    for (double t = 0; t < t_duration - 0.05; t += 0.05) {
      pos = traj.getPos(t);
      xs.push_back(pos(0));
      ys.push_back(pos(1));
      pos(2) += 0.0;

      zs.push_back(pos(2));
    }
    traj_vis.points.clear();

    for (int i = 0; i < zs.size(); i++) {
      pt.x = xs[i];
      pt.y = ys[i];
      // 轨迹显示的样子-----------------------------------------------------------------
      pt.z = zs[i];

      traj_vis.points.push_back(pt);
    }

    traj_vis_pub.publish(traj_vis);
  }


  void saveTraj(Trajectory traj) {
    geometry_msgs::Point pt, pt_last, pt_des, pt_des_last;
    Eigen::Vector3d pos;

    double t_duration = traj.getTotalDuration();
    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<double> zs;
    double last_zs;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ply(new pcl::PointCloud<pcl::PointXYZ>);  // 9.10
    for (double t = 0; t < t_duration - 0.05; t += 0.05) {
      pos = traj.getPos(t);
      xs.push_back(pos(0));
      ys.push_back(pos(1));
      pos(2) += 0.0;

      zs.push_back(pos(2));

    }

    for (int i = 0; i < zs.size(); i++) {
      pcl::PointXYZ cloud_point;
      pt.x = xs[i];
      pt.y = ys[i];
      pt.z = zs[i];
      cloud_point.x = pt.x;
      cloud_point.y = pt.y;
      cloud_point.z = pt.z;

      cloud_ply->points.push_back(cloud_point);
    }

    // 设置点云的宽度和高度
    cloud_ply->width = static_cast<uint32_t>(cloud_ply->points.size());
    cloud_ply->height = 1;
    cloud_ply->is_dense = false;

    // 保存点云为PLY文件
    std::string ply_file_path = "/home/hialb/Elastic-Tracker-main/ply_file.ply";
    pcl::io::savePLYFile(ply_file_path, *cloud_ply);
    std::cout << "---------------------------------Point cloud saved as " << ply_file_path << std::endl;
  }

  void pub_traj(const Trajectory& traj, const double& yaw, const ros::Time& stamp) {
    quadrotor_msgs::PolyTraj traj_msg;
    traj_msg.hover = false;
    traj_msg.order = 5;
    Eigen::VectorXd durs = traj.getDurations();
    int piece_num = traj.getPieceNum();
    traj_msg.duration.resize(piece_num);
    traj_msg.coef_x.resize(6 * piece_num);
    traj_msg.coef_y.resize(6 * piece_num);
    traj_msg.coef_z.resize(6 * piece_num);
    for (int i = 0; i < piece_num; ++i) {
      traj_msg.duration[i] = durs(i);
      CoefficientMat cMat = traj[i].getCoeffMat();
      int i6 = i * 6;
      for (int j = 0; j < 6; j++) {
        traj_msg.coef_x[i6 + j] = cMat(0, j);
        traj_msg.coef_y[i6 + j] = cMat(1, j);
        traj_msg.coef_z[i6 + j] = cMat(2, j);
      }
    }
    traj_msg.start_time = stamp;
    traj_msg.traj_id = traj_id_++;
    // NOTE yaw
    traj_msg.yaw = yaw;

    traj_pub_.publish(traj_msg);
  }

  void pub_mpc_traj(const Trajectory& traj){
    mpc::Polynome poly;
    Eigen::VectorXd ts = traj.getDurations();
    // std::cout << "traj.getDuration " << ts << std::endl;
    Eigen::MatrixXd poses = traj.getPositions();
    for(int i = 0; i< poses.cols(); i++){
      geometry_msgs::Point temp;
      temp.x = poses(0, i);
      temp.y = poses(1, i);
      temp.z = poses(2, i);
      poly.pos_pts.push_back(temp);
    }
    for(int i = 0; i < ts.size();i++){
      poly.t_pts.push_back(ts(i));
      double v = traj.getVel(ts(i)).norm();
      // std::cout << "refer v" <<v << std::endl;
    }

    poly.init_v.x = traj.getVel(0)[0];
    poly.init_v.y = traj.getVel(0)[1];
    poly.init_v.z = traj.getVel(0)[2];
    poly.init_a.x = traj.getAcc(0)[0];
    poly.init_a.y = traj.getAcc(0)[1];

    // poly.init_a.x = 0;
    // poly.init_a.y = 0;
    poly.init_a.z = traj.getAcc(0)[2];

    poly.start_time = ros::Time::now();

    mpc_traj_pub_.publish(poly);
    // std::cout << "pub poly" << std::endl;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "mpc_traj";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    for (int i = 0; i < poses.cols(); i++) {
        geometry_msgs::Point point;
        point.x = poses(0, i);
        point.y = poses(1, i);
        point.z = poses(2, i);
        marker.points.push_back(point);
    }

    mpc_vis_pub.publish(marker);

  }

  void triger_callback(const geometry_msgs::PoseStampedConstPtr& msgPtr) {

    goal_ << msgPtr->pose.position.x, msgPtr->pose.position.y, msgPtr->pose.position.z;  //------------------------------------------0.5
    std::cout << "triger callback-------------------------------" << std::endl;
    triger_received_ = true;

    // yaw (z-axis rotation)
    Eigen::Vector4d first_goal_ori = Eigen::Vector4d(msgPtr->pose.orientation.x, msgPtr->pose.orientation.y, msgPtr->pose.orientation.z, msgPtr->pose.orientation.w);
    double siny_cosp = 2 * (first_goal_ori(3) * first_goal_ori(2) + first_goal_ori(0) * first_goal_ori(1));
    double cosy_cosp = 1 - 2 * (first_goal_ori(1) * first_goal_ori(1) + first_goal_ori(2) * first_goal_ori(2));
    yaw_goal_temp = std::atan2(siny_cosp, cosy_cosp);
    // pose
    temp_goal_point = goal_;
  }

  // 11.11 fsm-----------------------
  void publish_cmd(int traj_id,
                   const Eigen::Vector3d& p,
                   const Eigen::Vector3d& v,
                   const Eigen::Vector3d& a,
                   double y, double yd) {
    quadrotor_msgs::PositionCommand cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "world";
    cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    cmd.trajectory_id = traj_id;

    cmd.position.x = p(0);
    cmd.position.y = p(1);
    cmd.position.z = 0.3;   // 12.19
    cmd.velocity.x = v(0);
    cmd.velocity.y = v(1);
    cmd.velocity.z = v(2);
    cmd.acceleration.x = a(0);
    cmd.acceleration.y = a(1);
    cmd.acceleration.z = a(2);
    cmd.yaw = y;
    cmd.yaw_dot = yd;
    pos_cmd_pub_.publish(cmd);
  }

  double err_yaw(double des_yaw, double odom_yaw) {
    if (des_yaw - odom_yaw >= 3.14159)
      return (des_yaw - odom_yaw) - 2 * 3.14159;
    else if (des_yaw - odom_yaw <= -3.14159)
      return 2 * 3.14159 + (des_yaw - odom_yaw);
    else
      return (des_yaw - odom_yaw);
  }
  // 11.29
  // 弧度转度数
  inline double rad2deg(double radians) {
    return radians * (180.0 / M_PI);
  }

  // 计算向量与主方向的角度
  double vectorAngle(const cv::Vec2f& unitVector, const cv::Vec2f& mainDirection) {
    float dotProduct = unitVector[0] * mainDirection[0] + unitVector[1] * mainDirection[1];
    float magnitudeUnitVector = norm(unitVector);
    if (magnitudeUnitVector == 0) return 0;
    double angleRad = acos(dotProduct / magnitudeUnitVector);  // 结果是弧度
    return rad2deg(angleRad);                                  // 转换为度数
  }

//12.2 TXT
void publishFirstGoal_txt(){
    if (!pos_queue.empty()) {
        Eigen::Vector3d first_goal_point = pos_queue.front();
        Eigen::Vector4d first_goal_ori = quat_queue.front();
        preset_goal_quat = quat_queue.front();
        goal_<< pos_queue.front()(0), pos_queue.front()(1),0.3;  // 12.19
        triger_received_ = true;
        // yaw (
        double siny_cosp = 2 * (first_goal_ori(3) * first_goal_ori(2) + first_goal_ori(0) * first_goal_ori(1));
        double cosy_cosp = 1 - 2 * (first_goal_ori(1) * first_goal_ori(1) + first_goal_ori(2) * first_goal_ori(2));
        yaw_goal_temp = std::atan2(siny_cosp, cosy_cosp);
        // pose 
        temp_goal_point = first_goal_point;
        temp_goal ++ ;

        

        // 可视化goal
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "world"; 
        goal.pose.position.x = pos_queue.front()(0);
        goal.pose.position.y = pos_queue.front()(1);
        goal.pose.position.z = 0.3;  // 12.19
        goal.pose.orientation.x = quat_queue.front()(0);
        goal.pose.orientation.y = quat_queue.front()(1);
        goal.pose.orientation.z = quat_queue.front()(2);
        goal.pose.orientation.w = quat_queue.front()(3);
        triger_pub.publish(goal); 

        preset_odom.pose.pose.position.x = pos_queue.front()(0);
        preset_odom.pose.pose.position.y = pos_queue.front()(1);
        preset_odom.pose.pose.position.z = 0.3;
        preset_odom.pose.pose.orientation.x = quat_queue.front()(0);
        preset_odom.pose.pose.orientation.y = quat_queue.front()(1);
        preset_odom.pose.pose.orientation.z = quat_queue.front()(2);
        preset_odom.pose.pose.orientation.w = quat_queue.front()(3);

        siyi1 = siyi_queue.front()(0);
        siyi2 = siyi_queue.front()(1);

        time_q1 = time_queue.front();

        
    }
    // 弹出第一个目标点
    if (!pos_queue.empty()) {
        pos_queue.pop();
        quat_queue.pop();
        siyi_queue.pop();
        time_queue.pop();

        // is_allow_to_start = 1;

        
    }
}

void publishNextGoal_txt(){
    if (!pos_queue.empty()) {
        goal_<< pos_queue.front()(0), pos_queue.front()(1),0.3;  // 12.19
        ROS_ERROR("[planner node] publishNextGoal...");
        triger_received_ = true;
        preset_goal_quat = quat_queue.front();
        // yaw (z-axis rotation)
        double siny_cosp = 2 * (quat_queue.front()(3) * quat_queue.front()(2) + quat_queue.front()(0) * quat_queue.front()(1));
        double cosy_cosp = 1 - 2 * (quat_queue.front()(1) * quat_queue.front()(1) + quat_queue.front()(2) * quat_queue.front()(2));
        yaw_goal_temp = std::atan2(siny_cosp, cosy_cosp);
        // pose 
        temp_goal_point = goal_;
        temp_goal ++ ;


        // 可视化goal
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "world"; 
        goal.pose.position.x = pos_queue.front()(0);
        goal.pose.position.y = pos_queue.front()(1);
        goal.pose.position.z = 0.3;  // 12.19
        goal.pose.orientation.x = quat_queue.front()(0);
        goal.pose.orientation.y = quat_queue.front()(1);
        goal.pose.orientation.z = quat_queue.front()(2);
        goal.pose.orientation.w = quat_queue.front()(3);
        triger_pub.publish(goal); 

        preset_odom.pose.pose.position.x = pos_queue.front()(0);
        preset_odom.pose.pose.position.y = pos_queue.front()(1);
        preset_odom.pose.pose.position.z = 0.3;
        preset_odom.pose.pose.orientation.x = quat_queue.front()(0);
        preset_odom.pose.pose.orientation.y = quat_queue.front()(1);
        preset_odom.pose.pose.orientation.z = quat_queue.front()(2);
        preset_odom.pose.pose.orientation.w = quat_queue.front()(3);


        siyi1 = siyi_queue.front()(0);
        siyi2 = siyi_queue.front()(1);
        time_q1 = time_queue.front();

      }
    // 弹出第一个目标点
    if (!pos_queue.empty()) {
      pos_queue.pop();
      quat_queue.pop();
      siyi_queue.pop();
      time_queue.pop();
      } else {
          ROS_INFO("[planning 12.2 node]: goal queue is empty");
          goal_<< 0,0,0.3; // 12.19
          triger_received_ = true;
          double siny_cosp = 0;
          double cosy_cosp = 1;
          yaw_goal_temp = std::atan2(siny_cosp, cosy_cosp);
          // pose 
          temp_goal_point = goal_;
          temp_goal =0 ;
          if_end = true;

          geometry_msgs::PoseStamped goal;
          goal.header.stamp = ros::Time::now();
          goal.header.frame_id = "world"; 
          goal.pose.position.x = 0;
          goal.pose.position.y = 0;
          goal.pose.position.z = 0.3; // 12.19
          goal.pose.orientation.x = 1;
          goal.pose.orientation.y = 0;
          goal.pose.orientation.z = 0;
          goal.pose.orientation.w = 0;
          triger_pub.publish(goal); 

      }
}

//  12.2 read TXT
  void  ReadtxtGoal(){
      int line_num = 0;

      std::string filename = "/home/hialb/Elastic-Tracker-main/odometry_data.txt";
      std::ifstream file(filename);

      if (!file.is_open()) {
          std::cerr << "Unable to open file: " << filename << std::endl;
          return;
      }
      else std::cout<< "open file successfully  -------------!!" << std::endl;  
      std::string line;

      // 读取文件中的每一行
      while (std::getline(file, line)) {
        line_num++;
        std::stringstream ss(line);
        std::string item;
        std::vector<double> row_pos_ori; // 用于存储单行数据的容器
        // 分割每一行的数据
        while (std::getline(ss, item, ',')) {
          double temp;
          std::istringstream iss(item);
          iss >> temp;
          row_pos_ori.push_back(temp);
        }
        time_queue.push(double(row_pos_ori[0]));
        pos_queue.push(Eigen::Vector3d(row_pos_ori[1], row_pos_ori[2], row_pos_ori[3]));
        quat_queue.push(Eigen::Vector4d(row_pos_ori[4], row_pos_ori[5], row_pos_ori[6],row_pos_ori[7]));
        siyi_queue.push(Eigen::Vector2d(row_pos_ori[8], row_pos_ori[9]));
        // siyi_queue.push(Eigen::Vector2d(row_pos_ori[8], row_pos_ori[9]));
        // std::cout<< row_pos_ori[1] << row_pos_ori[2] << row_pos_ori[3] << std::endl;
        row_pos_ori.clear();
      }
    
      // // 把原点push进去
      // pos_queue.push(Eigen::Vector3d(0, 0, 0.5));
      // quat_queue.push(Eigen::Vector4d(1, 0, 0,0));

      // 关闭文件
      file.close();
      std::cout<< "close file !!" << std::endl;  
      goal_num = line_num;
      temp_goal = 0;
      std::cout << "[Convert Pipe Node]: -----接收的目标点个数为：" << goal_num << std::endl;
   
  }

  void printFSMExecState() {
    static std::string state_str[10] = {"INIT","REC_MAP", "CHANGE_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "ROTATE", "DETECT", "STOP", "EMERGE_STOP"};

    std::cout << "[FSM]: state: " + state_str[int(exec_state_)] << std::endl;
  }

  void execFSMCallback(const ros::TimerEvent& e) {
    static int fsm_num = 0;
    fsm_num++;

    while (gridmap_lock_.test_and_set());
    gridmapPtr_->from_msg(map_msg_);
    replanStateMsg_.occmap = map_msg_;
    gridmap_lock_.clear();

    // NOTE local goal           （ 每个循环都实时更新当前这次的goal和odom ）

    Eigen::Vector3d delta = goal_ - odom_p;  // goal和当前点的相差

    if (fsm_num == 50) {
      printFSMExecState();
      if (!odom_received_)
        std::cout << "no odom." << std::endl;
      if (!triger_received_)
        std::cout << "wait for goal." << std::endl;
      fsm_num = 0;
    }

    switch (exec_state_) {
      case INIT: {
        if (!odom_received_ || !map_received_ || !triger_received_) {
          return;
        } else if (local_goal != goal_)
          // exec_state_ = REC_MAP;
          exec_state_ = CHANGE_TARGET;
        //---------------
        break;
      }

      case REC_MAP: {
        // NOTE obtain map
        while (gridmap_lock_.test_and_set());
        gridmapPtr_->from_msg(map_msg_);
        replanStateMsg_.occmap = map_msg_;
        gridmap_lock_.clear();
        exec_state_ = CHANGE_TARGET;
        //---------------
        break;
      }
      case CHANGE_TARGET: {
        ROS_WARN("[planner] CHANGE_TARGET...");
        if (delta.norm() < 15) {
          local_goal = goal_;
          std::cout << "delta.norm() < 15" << std::endl;
          std::cout << "local_goal? = " << local_goal << std::endl;
        } else {
          local_goal = delta.normalized() * 15 + odom_p;
          std::cout << "delta.norm() > 15" << std::endl;
          std::cout << "local_goal? = " << local_goal << std::endl;
        }
        exec_state_ = GEN_NEW_TRAJ;
        //---------------
        break;
      }
      case GEN_NEW_TRAJ: {  // 当收到goal的时候
        ROS_WARN("[planner] GEN_NEW_TRAJ...");
        Eigen::MatrixXd iniState;
        iniState.setZero(3, 3);
        ros::Time star_stamp = ros::Time::now() + ros::Duration(0.03);
        iniState.col(0) = odom_p;
        iniState.col(1) = Eigen::Vector3d(0, 0, 0);
        iniState.col(2) = Eigen::Vector3d(0, 0, 0);
        // NOTE path searching
        std::vector<Eigen::Vector3d> path;
        Eigen::Vector3d p_start = iniState.col(0);
        bool generate_new_traj_success = envPtr_->astar_search(p_start, local_goal, path);
        std::cout << "________local_goal:____" << local_goal << std::endl;
        Trajectory traj;
        if (generate_new_traj_success) {
          visPtr_->visualize_path(path, "astar");
          // NOTE corridor generating
          std::vector<Eigen::MatrixXd> hPolys;
          std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> keyPts;
          envPtr_->generateSFC(path, 2.0, hPolys, keyPts);
          envPtr_->visCorridor(hPolys);
          visPtr_->visualize_pairline(keyPts, "keyPts");

          // NOTE trajectory optimization
          Eigen::MatrixXd finState;
          finState.setZero(3, 3);
          finState.col(0) = path.back();
          finState.col(1) = Eigen::Vector3d(0, 0, 0);
          finState.col(2) = Eigen::Vector3d(0, 0, 0);
          // return;
          generate_new_traj_success = trajOptPtr_->generate_traj(iniState, finState, hPolys, traj);  // 更新出现在的traj
          visPtr_->visualize_traj(traj, "traj");
        }
        // NOTE collision check
        bool valid = false;
        if (generate_new_traj_success) {  // 如果成功generate轨迹
          valid = validcheck(traj, star_stamp);
        }
        if (valid) {
          emg_stop = false;
          stop_msg.data = false;
          stop_pub.publish(stop_msg);
          ROS_WARN("[planner] GEN_NEW_TRAJ SUCCESS");
          // NOTE : if the trajectory is known, watch that direction
          Eigen::Vector3d un_known_p = traj.getPos(traj.getTotalDuration() < 1.0 ? traj.getTotalDuration() : 1.0);
          Eigen::Vector3d dp = un_known_p - odom_p;
          double yaw = std::atan2(dp.y(), dp.x());
          pub_traj(traj, yaw, star_stamp);
          std::cout << "pub_traj.duration = " << traj.getDurations() << std::endl;
          pub_mpc_traj(traj);
          renderTraj(traj);
          // renderTraj2(traj);
          saveTraj(traj);
          traj_last = traj;
          last_start_stamp = star_stamp;
          traj_id_++;
          // Eigen::Vector3d p, v, a;
          // double t = star_stamp.toSec();
          // p = traj.getPos(t);
          // v = traj.getVel(t);
          // a = traj.getAcc(t);
          // publish_cmd(traj_id_, p, v, a, yaw, 0);
          exec_state_ = EXEC_TRAJ;
        } else {
          ROS_ERROR("[planner] GENERATE FAILED...EMERGE_STOP");
          exec_state_ = EMERGE_STOP;
        }
        //---------------
        break;
      }

      case EXEC_TRAJ: {
        ROS_WARN("[planner] EXEC_TRAJ...");
        double x_pos_error = temp_goal_point.x() - odom_p(0);
        double y_pos_error = temp_goal_point.y() - odom_p(1);
        double pos_err2 = std::sqrt(x_pos_error * x_pos_error + y_pos_error * y_pos_error);
        
        double t = (ros::Time::now() - last_start_stamp).toSec();
        double traj_t_rest = traj_last.getTotalDuration() - (ros::Time::now() - last_start_stamp).toSec();
        // bool new_goal = (local_goal - traj_last.getPos(traj_last.getTotalDuration())).norm() > tracking_dist_;
        Eigen::Vector3d diff = local_goal - traj_last.getPos(traj_last.getTotalDuration());
        double norm = sqrt(diff[0] * diff[0] + diff[1] * diff[1]); // 只计算前两个维度的平方和的平方根
        bool new_goal = norm > tracking_dist_;

        
        if (!new_goal) {
          if (traj_t_rest < 0.02 && pos_err2 < 0.25) {  // 到达上个目标点附近  12.17
            ROS_WARN("[planner] NEAR GOAL...");
            exec_state_ = ROTATE;
            //-----------------------------------------缺少状态转换？？？？？？？？？？？？？？？？
          } else if (validcheck(traj_last, last_start_stamp, traj_t_rest)) {
            // ROS_WARN("[planner] NO NEED REPLAN...");
            double t_delta = traj_last.getTotalDuration() < 1.0 ? traj_last.getTotalDuration() : 1.0;
            double t_yaw = (ros::Time::now() - last_start_stamp).toSec() + t_delta;
            Eigen::Vector3d un_known_p = traj_last.getPos(t_yaw);
            Eigen::Vector3d dp = un_known_p - odom_p;
            double yaw = std::atan2(dp.y(), dp.x());
            pub_traj(traj_last, yaw, last_start_stamp);
            //----------------------------------
            Eigen::Vector3d p, v, a;
            p = traj_last.getPos(t);
            v = traj_last.getVel(t);
            a = traj_last.getAcc(t);
            Eigen::Vector3d delta = p - odom_p;  // p和当前点的相差
            double planar_error = std::sqrt(delta.x() * delta.x() + delta.y() * delta.y());

            //-----------zhishi changshi yixia -------------

            if (delta.norm() > 0.9) {
              std::cout << "----------now delta -----" << planar_error << std::endl;
              exec_state_ = REPLAN_TRAJ;
            }

            renderTraj(traj_last);
            // pub_mpc_traj(traj_last);
            // renderTraj2(traj_last);
            // publish_cmd(traj_id_, p, v, a, yaw, 0);
          }
          else {
            exec_state_ = REPLAN_TRAJ;
          }
        } else{
          exec_state_ = REPLAN_TRAJ;
        }
        //---------------
        break;
      }

      case ROTATE: {
        ROS_WARN("[planner] ROTATE...");
        double x_pos_error = temp_goal_point.x() - odom_p(0);
        double y_pos_error = temp_goal_point.y() - odom_p(1);
        Eigen::Vector2d err_posxy = Eigen::Vector2d(x_pos_error, y_pos_error);
        Eigen::Vector2d forward_dir = Eigen::Vector2d(cos(odom_yaw_rad), sin(odom_yaw_rad));  // 当前机器人前进方向
        // double pos_err = forward_dir.dot(err_posxy);
        double pos_err = std::sqrt(x_pos_error * x_pos_error + y_pos_error * y_pos_error);
        double yaw_error = err_yaw(yaw_goal_temp, odom_yaw_rad);
        std::cout << "pos_err" << pos_err << std::endl;
        // if ((pos_err < 0.2) && (std::fabs(yaw_error) < 0.05)) {  // 到达目标点允许范围内
        //   std::cout << "before if_end----------"  << std::endl;
        //   if (if_end){
        //     exec_state_ = STOP;
        //   }else{
        //     exec_state_ = DETECT;
        //   }
        // } else {
        //   Eigen::Vector3d p, v, a;
        //   p = temp_goal_point;
        //   v = Eigen::Vector3d(0, 0, 0);
        //   a = Eigen::Vector3d(0, 0, 0);
        //   publish_cmd(traj_id_, p, v, a, yaw_goal_temp, 0);
        // }

        if (pos_err < 0.25) {  // 到达目标点允许范围内
          std::cout << "before if_end----------"  << std::endl;
          if (if_end){
            exec_state_ = STOP;
          }else{
            exec_state_ = DETECT;
          }
        } else {
          Eigen::Vector3d p, v, a;
          p = temp_goal_point;
          v = Eigen::Vector3d(0, 0, 0);
          a = Eigen::Vector3d(0, 0, 0);
          // publish_cmd(traj_id_, p, v, a, yaw_goal_temp, 0);
          // exec_state_ = EXEC_TRAJ;
        }

        //---------------
        break;
      }

      case REPLAN_TRAJ: {
        ROS_WARN("[planner] REPLAN_TRAJ...");
        Eigen::MatrixXd iniState;
        iniState.setZero(3, 3);
        ros::Time replan_stamp = ros::Time::now() + ros::Duration(0.03);
        iniState.col(0) = odom_p;                    // traj_poly_.getPos(replan_t);  9.9.9 ------------------------------------------------------------------------
        iniState.col(1) = Eigen::Vector3d(0, 0, 0);  // traj_poly_.getVel(replan_t); 9.9.9 ------------------------------------------------------------------------
        iniState.col(2) = Eigen::Vector3d(0, 0, 0);  // traj_poly_.getAcc(replan_t);  9.9.9 ------------------------------------------------------------------------
        std::vector<Eigen::Vector3d> path;
        Eigen::Vector3d p_start = iniState.col(0);
        bool generate_new_traj_success = envPtr_->astar_search(p_start, local_goal, path);
        std::cout << "________local_goal:____" << local_goal << std::endl;
        Trajectory traj;
        if (generate_new_traj_success) {
          visPtr_->visualize_path(path, "astar");
          // NOTE corridor generating
          std::vector<Eigen::MatrixXd> hPolys;
          std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> keyPts;
          envPtr_->generateSFC(path, 2.0, hPolys, keyPts);
          envPtr_->visCorridor(hPolys);
          visPtr_->visualize_pairline(keyPts, "keyPts");

          // NOTE trajectory optimization
          Eigen::MatrixXd finState;
          finState.setZero(3, 3);
          finState.col(0) = path.back();
          finState.col(1) = Eigen::Vector3d(0, 0, 0);  //--------------------------------------------------把末端速度改为0，0，0----------------- 9,9,13
          finState.col(2) = Eigen::Vector3d(0, 0, 0);  //--------------------------------------------------把末端速度改为0，0，0----------------- 9,9,13
          // return;
          generate_new_traj_success = trajOptPtr_->generate_traj(iniState, finState, hPolys, traj);  // 更新出现在的traj
          visPtr_->visualize_traj(traj, "traj");
        }
        // NOTE collision check
        bool valid = false;
        if (generate_new_traj_success) {
          valid = validcheck(traj, replan_stamp);
        }
        if (valid) {
          ROS_WARN("[planner] REPLAN SUCCESS");
          emg_stop = false;
          stop_msg.data = false;
          stop_pub.publish(stop_msg);
          ROS_WARN("[planner] GEN_NEW_TRAJ SUCCESS");
          // NOTE : if the trajectory is known, watch that direction
          Eigen::Vector3d un_known_p = traj.getPos(traj.getTotalDuration() < 1.0 ? traj.getTotalDuration() : 1.0);
          Eigen::Vector3d dp = un_known_p - odom_p;
          double yaw = std::atan2(dp.y(), dp.x());
          pub_traj(traj, yaw, replan_stamp);

          //------test---------------
          pub_mpc_traj(traj);


          renderTraj(traj);
          // renderTraj2(traj);
          saveTraj(traj);
          traj_last = traj;
          last_start_stamp = replan_stamp;
          traj_id_++;
          Eigen::Vector3d p, v, a;
          double t = replan_stamp.toSec();
          p = traj.getPos(t);
          v = traj.getVel(t);
          a = traj.getAcc(t);
          // publish_cmd(traj_id_, p, v, a, yaw, 0);
          exec_state_ = EXEC_TRAJ;
          
        }else {
          ROS_ERROR("[planner] REPLAN FAILED...EMERGE_STOP");
          exec_state_ = EMERGE_STOP;
        }

        //---------------
        break;
      }

      case DETECT: {
        ROS_WARN("[planner] DETECT.......");
        std::cout << "preset_goal_quat = " << preset_goal_quat << std::endl;
        double x_pos_error = temp_goal_point.x() - odom_p(0);
        double y_pos_error = temp_goal_point.y() - odom_p(1);
        Eigen::Vector2d err_posxy = Eigen::Vector2d(x_pos_error, y_pos_error);
        Eigen::Vector2d forward_dir = Eigen::Vector2d(cos(odom_yaw_rad), sin(odom_yaw_rad));  // 当前机器人前进方向
        double pos_err = forward_dir.dot(err_posxy);
        double yaw_error = err_yaw(yaw_goal_temp, odom_yaw_rad);
        double roll_interpolation, pitch_interpolation;
        
        if (local_goal == goal_) {  // 还没有换目标点
          detect_num++;
          std::cout << "STOP DETECT yaw ===============  :" << odom_yaw_rad << std::endl;
          Eigen::Vector3d p, v, a;
          ros::Rate rate(10); // 设置发布频率为10H
          bool angle_within_threshold = false;

          while (!angle_within_threshold) {
            // ros::spinOnce();
            ros::spinOnce();
            double roll_interpolation, pitch_interpolation;
            calculateInterpolation(roll_interpolation, pitch_interpolation);
            motion_msgs::Diablo_Ctrl dmsg;

            if (std::abs(roll_interpolation) > ANGLE_THRESHOLD || std::abs(pitch_interpolation) > ANGLE_THRESHOLD) {          
                dmsg.speed = 0.0;
                std::cout << "need to rotation " << std::endl;

                // 根据roll_interpolation的正负来设置msg.omega
                if (roll_interpolation > 0) {
                    dmsg.omega = 0.3;  // 右转
                } else {
                    dmsg.omega = -0.3; // 左转
                }
                std::cout << "Publishing rotation command to /adjust_yaw" << std::endl;

                rotation_cmd_pub_.publish(dmsg);
                // ros::Duration(0.1).sleep();
                } else {
                  dmsg.speed = 0.0; // 根据实际情况设置
                  dmsg.omega = 0.0;
                  std::cout << "Publishing stop rotation command to /adjust_yaw" << std::endl;
                  rotation_cmd_pub_.publish(dmsg);
                  angle_within_threshold = true;
                  std::cout << "Angle difference is below threshold, no rotation needed." << std::endl;
                    // 执行其他代码部分
                    // 这里可以添加其他代码逻辑
                }

                // loop_rate.sleep();
            }
          // calculateInterpolation(roll_interpolation, pitch_interpolation);
          // while(roll_interpolation >=  5 || roll_interpolation <= -5){
          //   std::string command = "python3 /home/hialb/Elastic-Tracker-main/src/diablo-test/diablo_ception/diablo_body/rotation.py " + std::to_string(roll_interpolation) + " " + std::to_string(pitch_interpolation);
          //   std::system(command.c_str());
          // }
          sleep(3);


          if (1){
            p = odom_p;
            v = Eigen::Vector3d(0, 0, 0);
            a = Eigen::Vector3d(0, 0, 0);
            // publish_cmd(traj_id_, p, v, a, yaw_goal_temp, 0);

          }else{
            p = odom_p;
            v = Eigen::Vector3d(0, 0, 0);
            a = Eigen::Vector3d(0, 0, 0);
            // publish_cmd(traj_id_, p, v, a, odom_yaw_rad, 0);
            stop_msg.data = true;
            stop_pub.publish(stop_msg);
          }
          
          if(detect_num ==1 && !detect_sim){
            current_file_name = int(time_q1);
            std::ostringstream oss;
            oss << current_file_name;




            std::string pythonScript0 = "/home/hialb/Elastic-Tracker-main/src/planning/planning/src/cap.py";

            std::string chmodCommand0 = "chmod +x " + pythonScript0;

            std::system(chmodCommand0.c_str());

            std::string command0 = "python " + pythonScript0 + " " + std::to_string(current_file_name);

            std::system(command0.c_str());

            std::cout << "enter here" <<std::endl;



            


            

            // 要执行的Python文件路径
            std::string pythonScript1 = "/home/hialb/Elastic-Tracker-main/src/unclear_trt/hk_predict2.py";
            // 要处理的单个图像文件路径
            // std::string imageFilePath = "/home/hialb/unclear_trt/demo_dpi4_s/777.jpg";
            // std::string imageFilePath = "/home/hialb/Elastic-Tracker-main/src/planning/planning/src/2222.jpg";
            std::string imageFilePath = "/home/hialb/Elastic-Tracker-main/src/planning/planning/src/" + std::to_string(current_file_name) +".jpg";

            std::string chmodCommand = "chmod +x " + pythonScript1;
            std::system(chmodCommand.c_str());

          //   // 提取Python文件名作为额外参数传递给Python脚本
            std::string pythonFileName = pythonScript1.substr(pythonScript1.find_last_of('/') + 1);

            // 构建Python命令，传递单个图像文件路径和Python文件名
            // std::string command1 = "python " + pythonScript1 + " " + imageFilePath + " " + pythonFileName;
            // 构建命令并执行
            std::string command1 = "python " + pythonScript1 + " --exp_name " + imageFilePath ;

            // std::cout << imageFilePath + " " + pythonFileName << std::endl;

            // 调用Python脚本
            int result = std::system(command1.c_str());

            if (result != 0) {
              std::cerr << "Error: Failed to execute Python script" << std::endl;
            } else {
              std::cout << "Python script executed successfully" << std::endl;
            }
          }

          // if (detect_num == 1 && !detect_sim) { 
            





          //   //std::string pythonScript00 = "/home/hialb/Elastic-Tracker-main/src/siyi/siyi_sdk/tests/set_angles.py";



          //   //std::string chmodCommand00 = "chmod +x " + pythonScript00;

          //   //std::system(chmodCommand00.c_str());

          //   // std::string command00 = "python " + pythonScript00 + " 10 " + "0";

          //   //std::string command00 = "python " + pythonScript00 + " " + std::to_string(siyi1) + " " + std::to_string(siyi2);

          //   //std::system(command00.c_str());

          //   //std::cout << "enter here" <<std::endl;


          //   std::string pythonScript0 = "/home/hialb/Elastic-Tracker-main/src/planning/planning/src/cap.py";

          //   std::string chmodCommand0 = "chmod +x " + pythonScript0;

          //   std::system(chmodCommand0.c_str());

          //   std::string command0 = "python " + pythonScript0;

          //   std::system(command0.c_str());

          //   std::cout << "enter here" <<std::endl;



            


            

          //   // 要执行的Python文件路径
          //   std::string pythonScript1 = "/home/hialb/Elastic-Tracker-main/src/unclear_trt/hk_predict2.py";
          //   // 要处理的单个图像文件路径
          //   // std::string imageFilePath = "/home/hialb/unclear_trt/demo_dpi4_s/777.jpg";
          //   std::string imageFilePath = "/home/hialb/Elastic-Tracker-main/src/planning/planning/src/2222.jpg";

          //   std::string chmodCommand = "chmod +x " + pythonScript1;
          //   std::system(chmodCommand.c_str());

          //   // 提取Python文件名作为额外参数传递给Python脚本
          //   std::string pythonFileName = pythonScript1.substr(pythonScript1.find_last_of('/') + 1);

          //   // 构建Python命令，传递单个图像文件路径和Python文件名
          //   // std::string command1 = "python " + pythonScript1 + " " + imageFilePath + " " + pythonFileName;
          //   // 构建命令并执行
          //   std::string command1 = "python " + pythonScript1 + " --exp_name " + imageFilePath ;

          //   // std::cout << imageFilePath + " " + pythonFileName << std::endl;

          //   // 调用Python脚本
          //   int result = std::system(command1.c_str());

          //   if (result != 0) {
          //     std::cerr << "Error: Failed to execute Python script" << std::endl;
          //   } else {
          //     std::cout << "Python script executed successfully" << std::endl;
          //   }


          //   // 图像文件路径
          //   std::string imageFilePath1 = "/home/hialb/Elastic-Tracker-main/hilab1.jpg";
          //   std::string imageFilePath2 = "/home/hialb/Elastic-Tracker-main/hilab2.jpg";


          //   // 要执行的Python文件路径
          //   std::string pythonScript2 = "/home/hialb/Elastic-Tracker-main/src/planning/planning/src/detect.py";

          //   // 构建命令并执行
          //   std::string command2 = "python " + pythonScript2 + " \"" + imageFilePath1 + "\" \"" + imageFilePath2 + "\"";
          //   int result__ = system(command2.c_str());

          //   if (result__ != 0) {
          //   std::cerr << "Error: Failed to execute Python script--2222" << std::endl;
          //   } else {
          //     std::cout << "Python script executed successfully--2222" << std::endl;
          //   }

          //   // 使用管道捕获输出
          //   std::array<char, 128> buffer;
          //   std::string result_;
          //   std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(command2.c_str(), "r"), pclose);
          //   if (!pipe) {
          //     std::cerr << "Error: popen() failed.\n";
          //   }
          //   while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
          //     result_ += buffer.data();
          //   }

          //   // 将结果解析为主方向向量
          //   std::vector<float> main_direction;
          //   std::stringstream ss(result_);
          //   float x, y;
          //   float average_length = 0.0;
          //   if (ss >> x >> y >> average_length) {
          //   // 成功读取主方向和平均长度
          //   main_direction.push_back(x);
          //   main_direction.push_back(y);
          //   std::cout << "主方向：[" << main_direction[0] << ", " << main_direction[1] << "]" << std::endl;
          //   std::cout << "平均长度：" << average_length << std::endl;
          // } else {
          //   std::cerr << "Error: Failed to parse main direction and average length from Python script output.\n";
          // }

          // }

          sleep(2);
          publishNextGoal_txt();
          exec_state_ = CHANGE_TARGET;
          // }
        } else
          // exec_state_ = REC_MAP;
          exec_state_ = CHANGE_TARGET;
          detect_num = 0;
        //---------------
        break;
      }

      case STOP: {
        ROS_WARN("[planner] STOP.......");
        if (local_goal == goal_) {
          std::cout << "STOP yaw ===============  :" << odom_yaw_rad << std::endl;
          Eigen::Vector3d p, v, a;
          p = odom_p;
          v = Eigen::Vector3d(0, 0, 0);
          a = Eigen::Vector3d(0, 0, 0);
          // publish_cmd(traj_id_, p, v, a, odom_yaw_rad, 0);
          stop_msg.data = true;
          stop_pub.publish(stop_msg);
        } else
          // exec_state_ = REC_MAP;
          exec_state_ = CHANGE_TARGET;
        //---------------
        break;
      }

      case EMERGE_STOP: {
        ROS_WARN("[planner] EMERGE_STOP.");
      
        // 1. 先让机器人保持安全状态（悬停/停下）
        stop_msg.data = true;
        stop_pub.publish(stop_msg);
      
        // 2. 检查一些恢复条件
        bool odom_in_map = gridmapPtr_->isInMap(odom_p);       // odom 是否在地图范围内
        bool odom_free   = !gridmapPtr_->isOccupied(odom_p);   // odom 是否不在障碍物里
        // bool sensor_ok   = !sensor_error_flag;                 // 传感器是否正常（你可以自己定义 sensor_error_flag）
      
        if (odom_in_map && odom_free) {
          ROS_WARN("[planner] EMERGE_STOP: conditions normal, retry replanning.");
          exec_state_ = REPLAN_TRAJ;  // ✅ 恢复后统一转回 REPLAN_TRAJ
        } 
        else {
          ROS_ERROR("[planner] EMERGE_STOP: waiting for recovery...");
          // 保持在 EMERGE_STOP，下一次循环再检查
          exec_state_ = EMERGE_STOP;
        }
      
        // 可选：加一个重试计数/超时逻辑
        static int emerge_retry_count = 0;
        emerge_retry_count++;
        if (emerge_retry_count > 50) { // 假设 50 次循环还没恢复
          ROS_ERROR("[planner] EMERGE_STOP: recovery failed, stopping task.");
          exec_state_ = STOP;
        }
      
        break;
      }
      


    }
  }

  void detect() {
  }

  std::string executePythonScript(const std::string& script, const std::string& args) {
    std::string command = "python " + script + " " + args;
    FILE* pipe = popen(command.c_str(), "r");
    if (!pipe) {
        std::cerr << "Failed to execute command." << std::endl;
        return "";
    }
    char buffer[128];
    std::string result = "";
    while (!feof(pipe)) {
        if (fgets(buffer, 128, pipe) != NULL)
            result += buffer;
    }
    pclose(pipe);
    return result;
  }

  void odom_callback(const nav_msgs::Odometry::ConstPtr& msgPtr) {
    while (odom_lock_.test_and_set());
    odom_msg_ = *msgPtr;
    current_odom = *msgPtr;
    odom_msg_.pose.pose.position.z = 0.35;  //--------------------------------------------
    odom_received_ = true;
    odom_lock_.clear();

    odom_p = Eigen::Vector3d(odom_msg_.pose.pose.position.x,
                             odom_msg_.pose.pose.position.y,
                             odom_msg_.pose.pose.position.z);
    odom_v = Eigen::Vector3d(odom_msg_.twist.twist.linear.x,
                             odom_msg_.twist.twist.linear.y,
                             odom_msg_.twist.twist.linear.z);

    odom_p(2) = -0.2;
    

    // 获取四元数
    double qx = msgPtr->pose.pose.orientation.x;
    double qy = msgPtr->pose.pose.orientation.y;
    double qz = msgPtr->pose.pose.orientation.z;
    double qw = msgPtr->pose.pose.orientation.w;

    // 创建四元数
    Eigen::Quaterniond quaternion(qw, qx, qy, qz);

    now_quat = Eigen::Vector4d(qx, qy, qz, qw);

    // 将四元数转换为旋转矩阵
    Eigen::Matrix3d rotation_matrix = quaternion.toRotationMatrix();

    // 从旋转矩阵获取偏航角（yaw）
    double yaw = atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));

    odom_yaw_rad = yaw;
  }

  void target_callback(const nav_msgs::Odometry::ConstPtr& msgPtr) {
    while (target_lock_.test_and_set());
    target_msg_ = *msgPtr;
    target_received_ = true;
    target_lock_.clear();
  }

  void gridmap_callback(const quadrotor_msgs::OccMap3dConstPtr& msgPtr) {
    while (gridmap_lock_.test_and_set());
    map_msg_ = *msgPtr;
    map_received_ = true;
    gridmap_lock_.clear();
  }

  void fake_timer_callback(const ros::TimerEvent& event) {
    heartbeat_pub_.publish(std_msgs::Empty());
    // 0.1.1 发布重规划目标点
    geometry_msgs::PoseStamped temp;
    temp.header.stamp = ros::Time::now();
    temp.pose.position.x = last_traj_t_rest_global_;  // 临时用来装时间
    last_traj_t_rest_pub_.publish(temp);

    if (!odom_received_ || !map_received_) {  // 位姿和地图接收确认
      return;
    }
    // obtain state of odom
    while (odom_lock_.test_and_set());
    auto odom_msg = odom_msg_;
    odom_lock_.clear();
    Eigen::Vector3d odom_p(odom_msg.pose.pose.position.x,
                           odom_msg.pose.pose.position.y,
                           odom_msg.pose.pose.position.z);
    Eigen::Vector3d odom_v(odom_msg.twist.twist.linear.x,
                           odom_msg.twist.twist.linear.y,
                           odom_msg.twist.twist.linear.z);
    if (!triger_received_) {  // 目标接收确认
      return;
    }
    // NOTE force-hover: waiting for the speed of drone small enough
    if (emg_stop && odom_v.norm() > 0.1) {  // 强制悬停且速度太大时，返回
      return;
    }

    // NOTE local goal
    Eigen::Vector3d local_goal;
    Eigen::Vector3d delta = goal_ - odom_p;
    if (delta.norm() < 15) {  // 当目标点和当前位姿相差没那么大时，直接local_goal = goal_规划
      local_goal = goal_;     // 这个15怎么定的？？？？
    } else {
      local_goal = delta.normalized() * 15 + odom_p;  // 否则，把先规划到当前目标点到那个方向的一部分
    }

    // NOTE obtain map                                                      //地图
    while (gridmap_lock_.test_and_set());
    gridmapPtr_->from_msg(map_msg_);
    replanStateMsg_.occmap = map_msg_;
    gridmap_lock_.clear();

    // NOTE determin whether to replan
    bool no_need_replan = false;
    if (!emg_stop && !wait_hover_) {                                                                         // 如果没有 【强制悬停】 或者 【等待悬停】 ；即【正常行进中】
                                                                                                             // std::cout<< "-------------DEBUG-----1111----------------"<<std::endl;
      double last_traj_t_rest = traj_poly_.getTotalDuration() - (ros::Time::now() - replan_stamp_).toSec();  // replan_stamp_是规划当前traj_poly_的开始时间
      // 0.1.1 增加重规划逻辑，发布这个数据
      last_traj_t_rest_global_ = last_traj_t_rest;
      bool new_goal = (local_goal - traj_poly_.getPos(traj_poly_.getTotalDuration())).norm() > tracking_dist_;  // 当local_goal和当前轨迹终点相差很大时，则有new_goal
                                                                                                                //     std::cout<<"  为什么没有new_goal????, traj_poly_时间戳末尾最后的点"<<traj_poly_.getPos(traj_poly_.getTotalDuration())<< std::endl;
      if (!new_goal) {                                                                                          // 如果没有new_goal
        //------------------------------------------------------------------------------------------------
        ROS_WARN("[planner] no new_goal...");
        if (last_traj_t_rest < 1.0) {  // 0.1               // 到达目标点附近，则啥都不干；还没有到达终点附近，且目标可达，则发布轨迹
          ROS_WARN("[planner] NEAR GOAL...");
          no_need_replan = true;
        } else if (validcheck(traj_poly_, replan_stamp_, last_traj_t_rest)) {
          ROS_WARN("[planner] NO NEED REPLAN...");
          double t_delta = traj_poly_.getTotalDuration() < 1.0 ? traj_poly_.getTotalDuration() : 1.0;
          double t_yaw = (ros::Time::now() - replan_stamp_).toSec() + t_delta;
          Eigen::Vector3d un_known_p = traj_poly_.getPos(t_yaw);
          Eigen::Vector3d dp = un_known_p - odom_p;
          double yaw = std::atan2(dp.y(), dp.x());
          pub_traj(traj_poly_, yaw, replan_stamp_);
          //------------------------------------------------------------
          Eigen::Vector3d p, v, a;
          double t = (ros::Time::now() - replan_stamp_).toSec();
          p = traj_poly_.getPos(t);
          v = traj_poly_.getVel(t);
          a = traj_poly_.getAcc(t);
          publish_cmd(traj_id_, p, v, a, yaw, 0);
          //-----------------------------------------------------------
          std::cout << "________renderTraj22222____,  local_goal: " << local_goal << std::endl;
          // std::cout<<"__________traj_poly______________"<<std::endl;
          // renderTraj(traj_poly_);
          // renderTraj2(traj_poly_);
          no_need_replan = true;
        }
      }
    }

    // NOTE determin whether to pub hover------------------------------------------------------------------------------------
    if ((goal_ - odom_p).norm() < tracking_dist_ + tolerance_d_ && odom_v.norm() < 0.1) {  // 当接近目标点且速度已经减小了的时候
      if (false) {
        if (!wait_hover_) {
          //      pub_hover_p(odom_p, ros::Time::now());   // 9.9.10
          wait_hover_ = true;
        }
        ROS_WARN("[planner] HOVERING...");
        std::cout << "______HOVERING..1111____, local_goal: " << local_goal << std::endl;
        replanStateMsg_.state = -1;
        replanState_pub_.publish(replanStateMsg_);
        return;
      }
    } else {
      wait_hover_ = false;
    }

    if (no_need_replan) {
      return;
    }

    // -------如果不需要replan就弹出函数了，运行到下面开始就代表状态需要replan

    // NOTE replan state
    Eigen::MatrixXd iniState;
    iniState.setZero(3, 3);
    ros::Time replan_stamp = ros::Time::now() + ros::Duration(0.03);  // replan_stamp 是当前时间的下一个小点
    double replan_t = (replan_stamp - replan_stamp_).toSec();
    if (emg_stop || replan_t > traj_poly_.getTotalDuration()) {  // 如果 【强制悬停】或者 【当前轨迹快超时了】，从悬停状态开始replan，否则从最新轨迹的当前点开始replan
      // should replan from the hover state
      iniState.col(0) = odom_p;
      iniState.col(1) = Eigen::Vector3d(0, 0, 0);  // odom_v; // 9.9.9 ------------------------------------------------------------------------
    } else {
      // should replan from the last trajectory
      iniState.col(0) = odom_p;                    // traj_poly_.getPos(replan_t);  9.9.9 ------------------------------------------------------------------------
      iniState.col(1) = Eigen::Vector3d(0, 0, 0);  // traj_poly_.getVel(replan_t); 9.9.9 ------------------------------------------------------------------------
      iniState.col(2) = Eigen::Vector3d(0, 0, 0);  // traj_poly_.getAcc(replan_t);  9.9.9 ------------------------------------------------------------------------
    }
    replanStateMsg_.header.stamp = ros::Time::now();
    replanStateMsg_.iniState.resize(9);
    Eigen::Map<Eigen::MatrixXd>(replanStateMsg_.iniState.data(), 3, 3) = iniState;

    // NOTE generate an extra corridor
    Eigen::Vector3d p_start = iniState.col(0);
    bool need_extra_corridor = iniState.col(1).norm() > 1.0;  // 如果起始速度大于1
    Eigen::MatrixXd hPoly;
    std::pair<Eigen::Vector3d, Eigen::Vector3d> line;
    if (need_extra_corridor) {
      Eigen::Vector3d v_norm = iniState.col(1).normalized();
      line.first = p_start;
      double step = 0.1;
      for (double dx = step; dx < 1.0; dx += step) {
        p_start += step * v_norm;
        if (gridmapPtr_->isOccupied(p_start)) {
          p_start -= step * v_norm;
          break;
        }
      }
      line.second = p_start;
      envPtr_->generateOneCorridor(line, 2.0, hPoly);
    }
    // NOTE path searching
    std::vector<Eigen::Vector3d> path;
    std::cout << "________debug why 2222222____, p_start:" << p_start << std::endl;
    bool generate_new_traj_success = envPtr_->astar_search(p_start, local_goal, path);
    std::cout << "________local_goal:____" << local_goal << std::endl;
    Trajectory traj;
    if (generate_new_traj_success) {
      visPtr_->visualize_path(path, "astar");
      // NOTE corridor generating
      std::vector<Eigen::MatrixXd> hPolys;
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> keyPts;
      envPtr_->generateSFC(path, 2.0, hPolys, keyPts);
      if (need_extra_corridor) {
        hPolys.insert(hPolys.begin(), hPoly);
        keyPts.insert(keyPts.begin(), line);
      }
      envPtr_->visCorridor(hPolys);
      visPtr_->visualize_pairline(keyPts, "keyPts");

      // NOTE trajectory optimization
      Eigen::MatrixXd finState;
      finState.setZero(3, 3);
      finState.col(0) = path.back();
      finState.col(1) = Eigen::Vector3d(0, 0, 0);  //--------------------------------------------------把末端速度改为0，0，0----------------- 9,9,13
      finState.col(2) = Eigen::Vector3d(0, 0, 0);  //--------------------------------------------------把末端速度改为0，0，0----------------- 9,9,13
      // return;
      generate_new_traj_success = trajOptPtr_->generate_traj(iniState, finState, hPolys, traj);  // 更新出现在的traj
      visPtr_->visualize_traj(traj, "traj");
    }

    // NOTE collision check
    bool valid = false;
    if (generate_new_traj_success) {
      valid = validcheck(traj, replan_stamp);
    } else {
      replanStateMsg_.state = -2;
      replanState_pub_.publish(replanStateMsg_);
    }
    if (valid) {
      emg_stop = false;
      ROS_WARN("[planner] REPLAN SUCCESS");
      replanStateMsg_.state = 0;
      replanState_pub_.publish(replanStateMsg_);
      // NOTE : if the trajectory is known, watch that direction
      Eigen::Vector3d un_known_p = traj.getPos(traj.getTotalDuration() < 1.0 ? traj.getTotalDuration() : 1.0);
      Eigen::Vector3d dp = un_known_p - odom_p;
      double yaw = std::atan2(dp.y(), dp.x());
      pub_traj(traj, yaw, replan_stamp);
      // renderTraj(traj);
      // renderTraj2(traj);
      saveTraj(traj);
      //-----------------------------------------------------------
      Eigen::Vector3d p, v, a;
      double t = replan_stamp.toSec();
      p = traj.getPos(t);
      v = traj.getVel(t);
      a = traj.getAcc(t);
      publish_cmd(traj_id_, p, v, a, yaw, 0);
      //----------------------------------------------------
      //   std::cout<<"________renderTraj11111____, local_goal:"<<local_goal<<std::endl;
      traj_poly_ = traj;
      replan_stamp_ = replan_stamp;
    } else if (emg_stop) {
      ROS_ERROR("[planner] REPLAN FAILED, HOVERING...");
      replanStateMsg_.state = 1;
      replanState_pub_.publish(replanStateMsg_);
      return;
    } else if (!validcheck(traj_poly_, replan_stamp_)) {
      emg_stop = true;
      ROS_FATAL("[planner] EMERGENCY STOP!!!");
      replanStateMsg_.state = 2;
      replanState_pub_.publish(replanStateMsg_);
      //      pub_hover_p(iniState.col(0), replan_stamp);
      return;
    } else {
      ROS_ERROR("[planner] REPLAN FAILED, EXECUTE LAST TRAJ...");
      replanStateMsg_.state = 3;
      replanState_pub_.publish(replanStateMsg_);
      return;  // current generated traj invalid but last is valid
    }
    visPtr_->visualize_traj(traj, "traj");
  }

  bool validcheck(const Trajectory& traj, const ros::Time& t_start, const double& check_dur = 1.0) {
    double t0 = (ros::Time::now() - t_start).toSec();
    t0 = t0 > 0.0 ? t0 : 0.0;
    double delta_t = check_dur < traj.getTotalDuration() ? check_dur : traj.getTotalDuration();
    for (double t = t0; t < t0 + delta_t; t += 0.01) {
      Eigen::Vector3d p = traj.getPos(t);
      if (gridmapPtr_->isOccupied(p)) {
        return false;
      }
    }
    return true;
  }

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

    // std::cout << "yaw = " << yaw << std::endl;
  }


  bool calculateInterpolation(double& roll_interpolation, double& pitch_interpolation) {
    double preset_roll, preset_pitch, preset_yaw;
    double current_roll, current_pitch, current_yaw;

    // 将预设的四元数转换为欧拉角
    quaternionToEuler(preset_odom.pose.pose.orientation, preset_roll, preset_pitch, preset_yaw);
    // 将当前的四元数转换为欧拉角
    quaternionToEuler(current_odom.pose.pose.orientation, current_roll, current_pitch, current_yaw);

    std::cout << "preset_yaw= " << preset_yaw << " current_yaw = " << current_yaw << "diff_yaw =" << preset_yaw - current_yaw << std::endl;
    

    // 计算插值
    roll_interpolation = preset_yaw - current_yaw;
    pitch_interpolation = preset_pitch - current_pitch;

    if(roll_interpolation > M_PI){
      roll_interpolation  = roll_interpolation - 2*M_PI;
    }else if(roll_interpolation < -M_PI){
      roll_interpolation  = roll_interpolation + 2*M_PI;
    }

    roll_interpolation = roll_interpolation * (180.0 / M_PI);
    pitch_interpolation = pitch_interpolation * (180.0 / M_PI);

    // std::cout << "roll = " << roll_interpolation << " pitch = " << pitch_interpolation << std::endl;


    // 判断是否小于阈值
    std::cout << "abs(roll_interpolation)= " << abs(roll_interpolation) << std::endl;
    return std::abs(roll_interpolation) < ANGLE_THRESHOLD;
  }



  void init(ros::NodeHandle& nh) {
    // set parameters of planning
    int plan_hz = 10;  // 10    9.9.9--------------------------------------------------------------------
    nh.getParam("plan_hz", plan_hz);
    nh.getParam("tracking_dur", tracking_dur_);
    nh.getParam("tracking_dist", tracking_dist_);
    nh.getParam("tolerance_d", tolerance_d_);
    nh.getParam("debug", debug_);
    nh.getParam("fake", fake_);
    nh.getParam("detect_sim", detect_sim);
    nh.getParam("virtual_ceiling_height", virtual_ceiling_height_); // 默认3米

    gridmapPtr_ = std::make_shared<mapping::OccGridMap>();
    envPtr_ = std::make_shared<env::Env>(nh, gridmapPtr_);
    visPtr_ = std::make_shared<visualization::Visualization>(nh);
    trajOptPtr_ = std::make_shared<traj_opt::TrajOpt>(nh);
    prePtr_ = std::make_shared<prediction::Predict>(nh);

    heartbeat_pub_ = nh.advertise<std_msgs::Empty>("heartbeat", 10);

    // 12.17
    triger_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

    ReadtxtGoal();


    gridmap_sub_ = nh.subscribe<quadrotor_msgs::OccMap3d>("gridmap_inflate", 1, &Nodelet::gridmap_callback, this, ros::TransportHints().tcpNoDelay());
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>("odom", 10, &Nodelet::odom_callback, this, ros::TransportHints().tcpNoDelay());
    // target_sub_ = nh.subscribe<nav_msgs::Odometry>("target", 10, &Nodelet::target_callback, this, ros::TransportHints().tcpNoDelay());

    // 11.11  fsm
    pos_cmd_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("position_cmd", 50);  //
    publishFirstGoal_txt();   //发布第一个目标点-===============================================================================
    exec_state_ = FSM_EXEC_STATE::INIT;

    // 11.26 siyi

    traj_pub_ = nh.advertise<quadrotor_msgs::PolyTraj>("trajectory", 5);
    mpc_traj_pub_ = nh.advertise<mpc::Polynome>("mpc_traj", 10);
    mpc_vis_pub = nh.advertise<visualization_msgs::Marker>("mpc_traj_vis", 10);
    traj_vis_pub = nh.advertise<visualization_msgs::Marker>("traj_vis", 10000);
    // traj_vis_pub = nh.advertise<visualization_msgs::Marker>("mpc_traj2", 10000);
    replanState_pub_ = nh.advertise<quadrotor_msgs::ReplanState>("replanState", 1);
    stop_pub = nh.advertise<std_msgs::Bool>("/stop_msg", 1000);
    // 0.1.1 重规划增加
    last_traj_t_rest_pub_ = nh.advertise<geometry_msgs::PoseStamped>("last_traj_t_rest_topic", 10);

    rotation_cmd_pub_ = nh.advertise<motion_msgs::Diablo_Ctrl>("adjust_yaw", 10);
    // sendCommands(rotation_cmd_pub_);

    if (debug_) {
      // TODO read debug data from files
      wr_msg::readMsg(replanStateMsg_, ros::package::getPath("planning") + "/../../../debug/replan_state.bin");
      inflate_gridmap_pub_ = nh.advertise<quadrotor_msgs::OccMap3d>("gridmap_inflate", 10);
      gridmapPtr_->from_msg(replanStateMsg_.occmap);
      prePtr_->setMap(*gridmapPtr_);
      std::cout << "plan state: " << replanStateMsg_.state << std::endl;
    } else if (fake_) {
      exec_timer_ = nh.createTimer(ros::Duration(0.1), &Nodelet::execFSMCallback, this);  // 0.05
      // plan_timer_ = nh.createTimer(ros::Duration(1.0 / plan_hz), &Nodelet::fake_timer_callback, this);
    }

    ROS_WARN("Planning node initialized!");
  }

 public:
  void onInit(void) {
    ros::NodeHandle nh(getMTPrivateNodeHandle());
    initThread_ = std::thread(std::bind(&Nodelet::init, this, nh));
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(planning::Nodelet, nodelet::Nodelet);