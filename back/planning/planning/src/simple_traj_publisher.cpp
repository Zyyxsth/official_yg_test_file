#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include <quadrotor_msgs/OccMap3d.h>
#include <mpc/Polynome.h>

#include <Eigen/Dense>
#include <atomic>
#include <memory>
#include <vector>
#include <string>

// ====== 你工程里的头文件（与 Nodelet 保持一致）======
#include <mapping/mapping.h>                 // mapping::OccGridMap
#include <env/env.hpp>                       // env::Env
#include <traj_opt/traj_opt.h>               // traj_opt::TrajOpt
#include <visualization/visualization.hpp>   // visualization::Visualization
#include <traj_opt/traj_opt.h>             // Trajectory & CoefficientMat

class SimpleOncePlanner {
public:
  SimpleOncePlanner(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh)
  {
    // 参数
    pnh_.param<std::string>("odom_topic",    odom_topic_,    std::string("/Robot_base_odom"));
    pnh_.param<std::string>("gridmap_topic", gridmap_topic_, std::string("gridmap_inflate"));
    pnh_.param<std::string>("traj_topic",    traj_topic_,    std::string("/target/planning/mpc_traj"));
    pnh_.param<std::string>("frame_id",      frame_id_,      std::string("world"));

    pnh_.param("use_goal_topic", use_goal_topic_, false);      // true：从 /move_base_simple/goal 取目标
    pnh_.param("goal_x", goal_[0], 5.0);
    pnh_.param("goal_y", goal_[1], 0.0);
    pnh_.param("goal_z", goal_[2], 0.0);

    pnh_.param("corridor_height", corridor_height_, 2.0);
    pnh_.param("start_delay",     start_delay_,     0.10);     // 给 MPC/通信一点对齐裕度

    // 实例化与话题
    gridmapPtr_ = std::make_shared<mapping::OccGridMap>();
    envPtr_     = std::make_shared<env::Env>(nh_, gridmapPtr_);
    visPtr_     = std::make_shared<visualization::Visualization>(nh_);
    trajOptPtr_ = std::make_shared<traj_opt::TrajOpt>(nh_);

    odom_sub_    = nh_.subscribe(odom_topic_,    1, &SimpleOncePlanner::odomCb,    this, ros::TransportHints().tcpNoDelay());
    gridmap_sub_ = nh_.subscribe(gridmap_topic_, 1, &SimpleOncePlanner::mapCb,     this, ros::TransportHints().tcpNoDelay());
    if (use_goal_topic_) {
      goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &SimpleOncePlanner::goalCb, this);
    }

    mpc_traj_pub_ = nh_.advertise<mpc::Polynome>(traj_topic_, 1, /*latched=*/true);
    vis_line_pub_ = nh_.advertise<visualization_msgs::Marker>("mpc_traj_vis", 1, true);

    ROS_INFO("[simple_traj_publisher] odom:%s  map:%s  traj_out:%s  frame:%s  goal: %s",
      odom_topic_.c_str(), gridmap_topic_.c_str(), traj_topic_.c_str(), frame_id_.c_str(),
      use_goal_topic_ ? "topic(/move_base_simple/goal)" : "params(goal_x/y/z)");
  }

  void spin() {
    ros::Rate r(50);
    while (ros::ok()) {
      ros::spinOnce();
      if (!published_ && has_odom_ && has_map_ && (has_goal_ || !use_goal_topic_)) {
        planOnce();
      }
      r.sleep();
    }
  }

private:
  // 一次性：A* -> SFC -> TrajOpt -> 发布 mpc::Polynome（+可视化）
  void planOnce() {
    if (published_) return;

    const Eigen::Vector3d start(odom_.x, odom_.y, odom_.z);
    const Eigen::Vector3d goal (goal_[0], goal_[1], goal_[2]);

    // ini / fin：与您 Nodelet 的 GEN_NEW_TRAJ 一致（v/a = 0）
    Eigen::MatrixXd iniState(3,3); iniState.setZero(); iniState.col(0) = start;
    Eigen::MatrixXd finState(3,3); finState.setZero(); finState.col(0) = goal;

    // 1) A* 路径
    std::vector<Eigen::Vector3d> path;
    bool ok = envPtr_->astar_search(iniState.col(0), finState.col(0), path);
    if (!ok || path.empty()) {
      ROS_ERROR("[simple_traj_publisher] astar_search failed.");
      return;
    }
    visPtr_->visualize_path(path, "astar");

    // 2) 生成走廊并可视化（用你项目里的接口）
    std::vector<Eigen::MatrixXd> hPolys;
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> keyPts;
    envPtr_->generateSFC(path, corridor_height_, hPolys, keyPts);
    envPtr_->visCorridor(hPolys);
    visPtr_->visualize_pairline(keyPts, "keyPts");

    // 3) 轨迹优化
    Trajectory traj;
    ok = trajOptPtr_->generate_traj(iniState, finState, hPolys, traj);
    if (!ok) {
      ROS_ERROR("[simple_traj_publisher] generate_traj failed.");
      return;
    }
    visPtr_->visualize_traj(traj, "traj");

    // 4) 发布给 MPC（严格按你原来的 mpc::Polynome 字段）
    pub_mpc_traj(traj);

    // 5) 再画一条独立的 LINE_STRIP（方便 RViz 直接看测试轨迹）
    draw_line_strip(traj);

    published_ = true;
    ROS_WARN("[simple_traj_publisher] DONE. published ONE latched trajectory.");
  }

  // ——与你原来的 pub_mpc_traj 一致（修复了 init_a 的维度）——
  void pub_mpc_traj(const Trajectory& traj){
    mpc::Polynome poly;

    Eigen::VectorXd ts    = traj.getDurations();   // 你原代码就是直接 push 到 t_pts
    Eigen::MatrixXd poses = traj.getPositions();   // (3, N)

    // pos_pts
    poly.pos_pts.reserve(poses.cols());
    for (int i = 0; i < poses.cols(); ++i) {
      geometry_msgs::Point p;
      p.x = poses(0, i);
      p.y = poses(1, i);
      p.z = poses(2, i);
      poly.pos_pts.push_back(p);
    }

    // t_pts（保持你的原语义）
    poly.t_pts.reserve(ts.size());
    for (int i = 0; i < ts.size(); ++i) {
      poly.t_pts.push_back(ts(i));
      // double v = traj.getVel(ts(i)).norm(); // 需要就打印
    }

    // 初始 v/a
    Eigen::Vector3d v0 = traj.getVel(0.0);
    Eigen::Vector3d a0 = traj.getAcc(0.0);
    poly.init_v.x = v0.x(); poly.init_v.y = v0.y(); poly.init_v.z = v0.z();
    poly.init_a.x = a0.x(); poly.init_a.y = a0.y(); poly.init_a.z = a0.z();

    // 给 MPC 一点起步延时（可配）
    poly.start_time = ros::Time::now() + ros::Duration(start_delay_);

    // 发布（latched）
    mpc_traj_pub_.publish(poly);

    // 顺手画一份（红色）——和你 nodelet 的风格一致
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp    = ros::Time::now();
    marker.ns = "mpc_traj";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;
    marker.points = poly.pos_pts;
    vis_line_pub_.publish(marker);
  }

  void draw_line_strip(const Trajectory& traj) {
    // 额外稠密采样画线（可直接在 RViz 看）
    visualization_msgs::Marker m;
    m.header.frame_id = frame_id_;
    m.header.stamp    = ros::Time::now();
    m.ns = "opt_traj_dense";
    m.id = 1;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.08;
    m.color.a = 1.0;
    m.color.r = 1.0; m.color.g = 0.5; m.color.b = 0.0;

    Eigen::VectorXd ts = traj.getDurations();
    double T = ts.size() ? ts(ts.size()-1) : 0.0;
    int N = std::max(100, (int)std::round(T / 0.02));
    for (int i = 0; i <= N; ++i) {
      double t = T * (double)i / (double)N;
      Eigen::Vector3d p = traj.getPos(t);
      geometry_msgs::Point q; q.x = p.x(); q.y = p.y(); q.z = p.z();
      m.points.push_back(q);
    }
    vis_line_pub_.publish(m);
  }

  // 回调
  void odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
    odom_ = msg->pose.pose.position;
    has_odom_ = true;
  }
  void mapCb(const quadrotor_msgs::OccMap3d::ConstPtr& msg) {
    gridmapPtr_->from_msg(*msg);
    has_map_ = true;
  }
  void goalCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    goal_[0] = msg->pose.position.x;
    goal_[1] = msg->pose.position.y;
    goal_[2] = msg->pose.position.z;
    has_goal_ = true;
    ROS_INFO("[simple_traj_publisher] got goal: (%.3f, %.3f, %.3f)", goal_[0], goal_[1], goal_[2]);
  }

private:
  ros::NodeHandle nh_, pnh_;
  // topics & params
  std::string odom_topic_, gridmap_topic_, traj_topic_, frame_id_;
  bool use_goal_topic_{false};
  double goal_[3]{5.0, 0.0, 0.0};
  double corridor_height_{2.0};
  double start_delay_{0.10};

  // ros
  ros::Subscriber odom_sub_, gridmap_sub_, goal_sub_;
  ros::Publisher  mpc_traj_pub_;
  ros::Publisher  vis_line_pub_;

  // data flags
  geometry_msgs::Point odom_;
  bool has_odom_{false}, has_map_{false}, has_goal_{false}, published_{false};

  // core modules
  std::shared_ptr<mapping::OccGridMap>          gridmapPtr_;
  std::shared_ptr<env::Env>                     envPtr_;
  std::shared_ptr<visualization::Visualization> visPtr_;
  std::shared_ptr<traj_opt::TrajOpt>            trajOptPtr_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_traj_publisher");
  ros::NodeHandle nh, pnh("~");
  SimpleOncePlanner node(nh, pnh);
  node.spin();
  return 0;
}
