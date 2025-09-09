#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PolyTraj.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>

#include <traj_opt/poly_traj_utils.hpp>

ros::Publisher pos_cmd_pub_;
ros::Time heartbeat_time_;
bool receive_traj_ = false;
bool flight_start_ = false;
quadrotor_msgs::PolyTraj trajMsg_, trajMsg_last_;
Eigen::Vector3d last_p_;
double last_yaw_ = 0;

void publish_cmd(int traj_id,
                 const Eigen::Vector3d &p,
                 const Eigen::Vector3d &v,
                 const Eigen::Vector3d &a,
                 double y, double yd) {
  quadrotor_msgs::PositionCommand cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id;

  cmd.position.x = p(0);
  cmd.position.y = p(1);
  cmd.position.z = p(2);
  cmd.velocity.x = v(0);
  cmd.velocity.y = v(1);
  cmd.velocity.z = v(2);
  cmd.acceleration.x = a(0);
  cmd.acceleration.y = a(1);
  cmd.acceleration.z = a(2);
  cmd.yaw = y;
  cmd.yaw_dot = yd;
  pos_cmd_pub_.publish(cmd);
  last_p_ = p;
}

// void publish_cmd_stop() {
//   quadrotor_msgs::PositionCommand cmd;
//   cmd.header.stamp = ros::Time::now();
//   cmd.header.frame_id = "world";
//   cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
//   cmd.trajectory_id = 0;

//   cmd.position.x =  odom_p.x();
//   cmd.position.y =  odom_p.y();
//   cmd.position.z =  odom_p.z();
//   cmd.velocity.x =  0;
//   cmd.velocity.y = 0;
//   cmd.velocity.z = 0;
//   cmd.acceleration.x = 0;
//   cmd.acceleration.y = 0;
//   cmd.acceleration.z = 0;
//   cmd.yaw = odom_yaw_rad;
//   cmd.yaw_dot = 0;
//   pos_cmd_pub_.publish(cmd);
// //  last_p_ = odom_p;
// }


bool exe_traj(const quadrotor_msgs::PolyTraj &trajMsg) {
  double t = (ros::Time::now() - trajMsg.start_time).toSec();
  if (t > 0) {
    if (trajMsg.hover) {
      if (trajMsg.hover_p.size() != 3) {
        ROS_ERROR("[publishn cmd] hover_p is not 3d!");
      }
      Eigen::Vector3d p, v0;
      p.x() = trajMsg.hover_p[0];
      p.y() = trajMsg.hover_p[1];
      p.z() = trajMsg.hover_p[2];
      v0.setZero();
      publish_cmd(trajMsg.traj_id, p, v0, v0, last_yaw_, 0);  // TODO yaw
      ROS_ERROR("[publish cmd] publish cmd 111111111111111111111!");
      return true;
    }
    if (trajMsg.order != 5) {
      ROS_ERROR("[publish cmd] Only support trajectory order equals 5 now!");
      return false;
    }
    if (trajMsg.duration.size() * (trajMsg.order + 1) != trajMsg.coef_x.size()) {
      ROS_ERROR("[publish cmd] WRONG trajectory parameters!");
      return false;
    }
    int piece_nums = trajMsg.duration.size();
    std::vector<double> dura(piece_nums);
    std::vector<CoefficientMat> cMats(piece_nums);
    for (int i = 0; i < piece_nums; ++i) {
      int i6 = i * 6;
      cMats[i].row(0) << trajMsg.coef_x[i6 + 0], trajMsg.coef_x[i6 + 1], trajMsg.coef_x[i6 + 2],
          trajMsg.coef_x[i6 + 3], trajMsg.coef_x[i6 + 4], trajMsg.coef_x[i6 + 5];
      cMats[i].row(1) << trajMsg.coef_y[i6 + 0], trajMsg.coef_y[i6 + 1], trajMsg.coef_y[i6 + 2],
          trajMsg.coef_y[i6 + 3], trajMsg.coef_y[i6 + 4], trajMsg.coef_y[i6 + 5];
      cMats[i].row(2) << trajMsg.coef_z[i6 + 0], trajMsg.coef_z[i6 + 1], trajMsg.coef_z[i6 + 2],
          trajMsg.coef_z[i6 + 3], trajMsg.coef_z[i6 + 4], trajMsg.coef_z[i6 + 5];

      dura[i] = trajMsg.duration[i];
    }
    Trajectory traj(dura, cMats);
    if (t > traj.getTotalDuration()) {
      // ROS_ERROR("[publish cmd] trajectory too short left!");
      return false;
    }
    Eigen::Vector3d p, v, a;
    p = traj.getPos(t);
    v = traj.getVel(t);
    a = traj.getAcc(t);
    // NOTE yaw
    double yaw = trajMsg.yaw;
    double d_yaw = yaw - last_yaw_;
    d_yaw = d_yaw >= M_PI ? d_yaw - 2 * M_PI : d_yaw;
    d_yaw = d_yaw <= -M_PI ? d_yaw + 2 * M_PI : d_yaw;
    double d_yaw_abs = fabs(d_yaw);
    if (d_yaw_abs >= 0.02) {
      yaw = last_yaw_ + d_yaw / d_yaw_abs * 0.02;  //--------------------------------------------------------------------------------------9.9.10 11.1debug
    }
    publish_cmd(trajMsg.traj_id, p, v, a, yaw, 0);  // TODO yaw
  //  ROS_ERROR("[publish cmd] publish cmd 2222222222222222222222!");
    last_yaw_ = yaw;
    return true;
  }
  return false;
}

void heartbeatCallback(const std_msgs::EmptyConstPtr &msg) {
  heartbeat_time_ = ros::Time::now();
}

void polyTrajCallback(const quadrotor_msgs::PolyTrajConstPtr &msgPtr) {
  trajMsg_ = *msgPtr;
  if (!receive_traj_) {
    trajMsg_last_ = trajMsg_;
    receive_traj_ = true;
  }
}

void cmdCallback(const ros::TimerEvent &e) {
  if (!receive_traj_) {
    ROS_ERROR_ONCE("[publish cmd] receive_traj_? NO!!!!!!!!!!!!!!!!!!!!!!");
    return;
  }
  ros::Time time_now = ros::Time::now();
    if ((time_now - heartbeat_time_).toSec() > 0.5) {    //表示没有再接收到轨迹的规划--------999   0.5
   // std::cout<<"(time_now - heartbeat_time_).toSec()"<< (time_now - heartbeat_time_).toSec()<< std::endl;
    ROS_ERROR_ONCE("[publish cmd] Lost heartbeat from the planner, is he dead?");
    publish_cmd(trajMsg_.traj_id, last_p_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0, 0);  // TODO yaw
    ROS_ERROR("[publish cmd] publish cmd 333333333333333333333333!");
    return;
  }
  if (exe_traj(trajMsg_)) {
    trajMsg_last_ = trajMsg_;
    return;
  } else if (exe_traj(trajMsg_last_)) {
   return;
  }
}

// // 0.1.4 接收实际的odom
// void odom_callbackttt(const nav_msgs::Odometry::ConstPtr& msgPtr) {
//     odom_msg_ttt = *msgPtr;
//  //   odom_received_ = true;

//     // rcv_odom_stamp = ros::Time::now();

//     odom_pttt.x() = odom_msg_ttt.pose.pose.position.x;
//     odom_pttt.y() = odom_msg_ttt.pose.pose.position.y;
//     odom_pttt.z() = odom_msg_ttt.pose.pose.position.z;

//     odom_vttt.x() = odom_msg_ttt.twist.twist.linear.x;
//     odom_vttt.y() = odom_msg_ttt.twist.twist.linear.y;
//     odom_vttt.z() = odom_msg_ttt.twist.twist.linear.z;
    
//     // 获取四元数
//     double qx = msgPtr->pose.pose.orientation.x;
//     double qy = msgPtr->pose.pose.orientation.y;
//     double qz = msgPtr->pose.pose.orientation.z;
//     double qw = msgPtr->pose.pose.orientation.w;

//     // 创建四元数
//     Eigen::Quaterniond quaternion(qw, qx, qy, qz);
    
//     // 将四元数转换为旋转矩阵
//     Eigen::Matrix3d rotation_matrix = quaternion.toRotationMatrix();
    
//     // 从旋转矩阵获取偏航角（yaw）
//     double yaw = atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));
    
//     // 将偏航角转换到 0 到 360 度的范围
//     // odom_yaw_deg = normalize_yaw(yaw);

//     // 如果用的是弧度，把上面这两行注释换成下面这个
//     odom_yaw_radttt = yaw;
// }


int main(int argc, char **argv) {
  ros::init(argc, argv, "publish cmd");
  ros::NodeHandle nh("~");

  ros::Subscriber poly_traj_sub = nh.subscribe("trajectory", 50, polyTrajCallback);//10
  ros::Subscriber heartbeat_sub = nh.subscribe("heartbeat", 50, heartbeatCallback);//10

  // ros::Subscriber odom_sub_ttt = nh.subscribe<nav_msgs::Odometry>("/new_odom", 10, odom_callbackttt);;//10 

  pos_cmd_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("position_cmd", 50); //

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);

  ros::Duration(1.0).sleep();
  ROS_WARN("[publish cmd]: ready.");

  ros::spin();

  return 0;
}