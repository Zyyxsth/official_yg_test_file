#include "so3_controller/convert_pipe_node.h"
#include "motion_msgs/Diablo_Ctrl.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>   // 对于 std::fabs

// 0.1.2 适配轮腿
#include <geometry_msgs/Point.h>
#include <unistd.h>

using namespace class_convert_pipe;
using namespace std;


/*#####################*/
/* Message struct      */
/* 消息类型列表：        */
/* 1：xyz速度，wxyz姿态  */
/*#####################*/
int msgid;
bool readyToSendPipeMessage = false; // 当标志位为true运行管道通信
#define MSGKEY 1001
#define MSGTYPE 1 // 发送过去的状态标志
double state_number = 0; // 发送的控制指令最后一位，记录状态
struct pipeMessageBuffer{
    long mType; // msg type
    double data[9]; // calocity xyz, vx,vy,vz,yaw,yaw_dot, state_number
};
std::queue<Eigen::Vector3d> move_base_goal_queue_; 
std::queue<Eigen::Vector3d> pos_queue;
//std::queue<Eigen::Quaterniond> quat_queue;// 储存目标的四元数
std::queue<Eigen::Vector4d> quat_queue;// 储存目标的四元数
Eigen::Vector3d temp_goal_point;  //当前的goal是哪一个
double yaw_goal_temp;
int goal_num = 0;  //看有多少个goal
int temp_goal;

bool pid_near = false;
bool stop_car = false;


// 订阅position_cmd
ros::Subscriber position_cmd_subscriber_;
// 0.1.1 重规划目标点接收
ros::Subscriber last_traj_t_rest_subscriber_;
double last_traj_t_rest_global;
geometry_msgs::PoseStamped last_traj_t_rest_msg;
int is_allow_to_start;
ros::Publisher triger_pub;
// 全局保存的position_cmd
quadrotor_msgs::PositionCommand cmd_msg;
// 使用定时器判断是否有消息进来
ros::Time rcv_last_cmd_stamp;
// 全局保存的trajectory id，判断是否更新
int trajectory_id_old = -1;
int trajectory_id_new = 0; // 这两个没啥用


ros::Publisher diablo_cmd_pub; //diablo话题发布
motion_msgs::Diablo_Ctrl cmd_dia;
geometry_msgs::Point stop_msg_; //发布停止话题

// ********************************************
// 命令参数
// *******************************************
Eigen::Vector3d cmd_p;
Eigen::Vector3d cmd_v;
Eigen::Vector3d cmd_a;
Eigen::Vector3d cmd_j;
double cmd_yaw;
double cmd_yaw_rate;
// 用户的目标点
double p0x;
double p1x;
double p2x;
double p3x;
double p4x;
double p0y;
double p1y;
double p2y;
double p3y;
double p4y;
double p0z;
double p1z;
double p2z;
double p3z;
double p4z;
double p5x, p5y, p5z;

double ori0x , ori1x , ori2x , ori3x , ori4x ;
double ori0y , ori1y , ori2y , ori3y , ori4y ;
double ori0z , ori1z , ori2z , ori3z , ori4z ;
double ori0w , ori1w , ori2w , ori3w , ori4w ;
double ori5x , ori5y , ori5z , ori5w ;

// std::queue<Eigen::Vector3d> move_base_goal_queue_; // 使用 Eigen::Vector3d 存储目标点
// 多个点情况下，成功到达每一个点之后的标志位
bool is_reach_one_goal_ = false;
// 允许发送下一个目标点指令
bool is_allow_send_next_point = false;

// 0.1.2 适配轮腿
ros::Subscriber wheel_leg_pos_cmd_subscriber;
ros::Publisher wheel_leg_pos_cmd_pub;
ros::Subscriber wheel_leg_yaw_cmd_subscriber;
ros::Publisher wheel_leg_yaw_cmd_pub;
ros::Publisher stop_pub;
double wheel_leg_pos_x = 0.0;
double wheel_leg_pos_y = 0.0;
double wheel_leg_pos_z = 0.0;
double wheel_leg_yaw = 0.0;
double wheel_leg_yaw_dot = 0.0;
double wheel_leg_state_number = 0.0;
geometry_msgs::Point wheel_leg_pos_cmd_msg;
geometry_msgs::Point wheel_leg_yaw_cmd_msg;

// 0.1.4 移植控制器给轮腿使用
// ********************************************
// 控制器参数
// ********************************************
double kp0;
double kp1;
double kp2;
double ki0;
double ki1;
double ki2;
double kd0;
double kd1;
double kd2;

double kv0;
double kv1;
double kv2;

double v_max_x;
double v_max_y;
double v_max_z;

double a_max_x;
double a_max_y;
double a_max_z;

double pid_yaw_kp,pid_yaw_ki,pid_yaw_kd,pid_yaw_kv,vmax_yaw,amax_yaw;

double ctrl_frequency;// 控制频率 is_use_simulation_control
int is_use_simulation_control;// 是否使用PID控制器
double max_pos_err; // 最大位置误差，限幅

// 9.9.9
bool if_txtdot; // 是否打点
// 实例化PID控制器
px4_ctrl::PIDctrl pid_ctrl_x;
px4_ctrl::PIDctrl pid_ctrl_y;
px4_ctrl::PIDctrl pid_ctrl_z;
px4_ctrl::PIDctrl pid_ctrl_yaw_near;
// 实际的odom
ros::Subscriber odom_sub_;
nav_msgs::Odometry odom_msg_;
std::atomic_bool odom_received_ = ATOMIC_VAR_INIT(false);
ros::Time rcv_odom_stamp; // TODO：减去当前时间
// 当前实际的位置和速度
Eigen::Vector3d odom_p;
Eigen::Vector3d odom_v;
double odom_yaw_deg;
double odom_yaw_rad;
double odom_yaw_dot;

// 0.1.5 预留用户航点接口
// 示例数据，确保总数为3的倍数
// 0.1.6 无人机仿真高度加高
std::vector<double> user_keyboard_input_points = {
    8.0, -24.0, 4.5,
    4.5, -38.5, 5.5,
    0.0, 0.0, 3.0
};
int is_user_keyboard_input_points_success = 0;


// 发布第一个点，第一个点是0 0 1.5预备高度
void publishFirstGoal() {
    if (!move_base_goal_queue_.empty()) {
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "world"; // 设置目标点的参考坐标系 还是说发布的是local需要明确
        goal.header.stamp = ros::Time::now();
        Eigen::Vector3d first_goal_point = move_base_goal_queue_.front();
        goal.pose.position.x = first_goal_point(0);
        goal.pose.position.y = first_goal_point(1);
        goal.pose.position.z = first_goal_point(2);
        triger_pub.publish(goal); // 发布第一个目标点
    }
    // 弹出第一个目标点
    //9.9.10
   // sleep(2);
    if (!move_base_goal_queue_.empty()) {
        move_base_goal_queue_.pop();
     //   ROS_INFO("[convert pirp node]: reach the first goal height.");
        // 0.1.1
        is_allow_to_start = 1;
    }
}

// 发布下一个点
void publishNextGoal() {
    if (!move_base_goal_queue_.empty()) {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "world"; 
        goal.pose.position.x = move_base_goal_queue_.front()(0);
        goal.pose.position.y = move_base_goal_queue_.front()(1);
        goal.pose.position.z = move_base_goal_queue_.front()(2);
        triger_pub.publish(goal); 
    }
    // 弹出第一个目标点
    if (!move_base_goal_queue_.empty()) {
      move_base_goal_queue_.pop();
    } else {
        ROS_INFO("[convert pirp node]: goal queue is empty");
    }
}

// 9.9.9 TXT
void publishFirstGoal_txt(){
    if (!pos_queue.empty()) {
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "world"; // 设置目标点的参考坐标系 还是说发布的是local需要明确
        goal.header.stamp = ros::Time::now();
        Eigen::Vector3d first_goal_point = pos_queue.front();
        Eigen::Vector4d first_goal_ori = quat_queue.front();
        goal.pose.position.x = first_goal_point(0);
        goal.pose.position.y = first_goal_point(1);
        goal.pose.position.z = 0.5;
        goal.pose.orientation.x = first_goal_ori(0);
        goal.pose.orientation.y = first_goal_ori(1);
        goal.pose.orientation.z = first_goal_ori(2);
        goal.pose.orientation.w = first_goal_ori(3);
        triger_pub.publish(goal); // 发布第一个目标点

//9.9.10
        // yaw (z-axis rotation)
        double siny_cosp = 2 * (first_goal_ori(3) * first_goal_ori(2) + first_goal_ori(0) * first_goal_ori(1));
        double cosy_cosp = 1 - 2 * (first_goal_ori(1) * first_goal_ori(1) + first_goal_ori(2) * first_goal_ori(2));
        yaw_goal_temp = std::atan2(siny_cosp, cosy_cosp);
        // pose 
        temp_goal_point = first_goal_point;
        temp_goal ++ ;

    }
    // 弹出第一个目标点
    sleep(2);
    if (!pos_queue.empty()) {
        pos_queue.pop();
        quat_queue.pop();
        //ROS_INFO("[convert pirp node]: reach the first goal height.");
        // 0.1.1
        is_allow_to_start = 1;
    }
}

//9.9.9 TXT
void publishNextGoal_txt(){
    if (!pos_queue.empty()) {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "world"; 
        goal.pose.position.x = pos_queue.front()(0);
        goal.pose.position.y = pos_queue.front()(1);
        goal.pose.position.z = 0.5;
        goal.pose.orientation.x = quat_queue.front()(0);
        goal.pose.orientation.y = quat_queue.front()(1);
        goal.pose.orientation.z = quat_queue.front()(2);
        goal.pose.orientation.w = quat_queue.front()(3);
        triger_pub.publish(goal); 

//9.9.10
        // yaw (z-axis rotation)
        double siny_cosp = 2 * (quat_queue.front()(3) * quat_queue.front()(2) + quat_queue.front()(0) * quat_queue.front()(1));
        double cosy_cosp = 1 - 2 * (quat_queue.front()(1) * quat_queue.front()(1) + quat_queue.front()(2) * quat_queue.front()(2));
        yaw_goal_temp = std::atan2(siny_cosp, cosy_cosp);
        // pose 
        temp_goal_point = pos_queue.front();
        temp_goal ++ ;
    }
    // else {  // 当没有goal了之后  Go back to start point
    //     geometry_msgs::PoseStamped goal;
    //     goal.header.stamp = ros::Time::now();
    //     goal.header.frame_id = "world"; 
    //     goal.pose.position.x = 0;
    //     goal.pose.position.y = 0;
    //     goal.pose.position.z = 0.5;
    //     goal.pose.orientation.x = 1;
    //     goal.pose.orientation.y = 0;
    //     goal.pose.orientation.z = 0;
    //     goal.pose.orientation.w = 0;
    //     triger_pub.publish(goal); 

    // //9.9.10
    //     // yaw (z-axis rotation)
    //     double siny_cosp = 2 * (quat_queue.front()(3) * quat_queue.front()(2) + quat_queue.front()(0) * quat_queue.front()(1));
    //     double cosy_cosp = 1 - 2 * (quat_queue.front()(1) * quat_queue.front()(1) + quat_queue.front()(2) * quat_queue.front()(2));
    //     yaw_goal_temp = 0;
    //     // pose 
    //     temp_goal_point = Eigen::Vector3d(goal.pose.position.x,goal.pose.position.y,goal.pose.position.z);
    //     temp_goal ++ ;
    //     ROS_WARN("[convert pirp node]: Go back to start point!!");
    // }
    // 弹出第一个目标点
    if (!pos_queue.empty()) {
      pos_queue.pop();
      quat_queue.pop();
    } else {
        ROS_INFO("[convert pirp node]: goal queue is empty");
    }
}

/*################*/
/*  Constructor   */
/*################*/
ConvertPipeNode::ConvertPipeNode(ros::NodeHandle nh): nh_(nh) {

  initVariable();

  initServiceClient();
  initSubscriber();

  ros::Duration(1.0).sleep();

  initServiceServer();
  initPublisher();

  initTimer();
}
 
/*##############*/
/*  Destructor  */
/*##############*/

ConvertPipeNode::~ConvertPipeNode(){
}

bool ConvertPipeNode::initVariable(){

//   printf("Loading Params Finish\n");
//   printf("---------------------\n");

  

//   ROS_INFO_STREAM("Convert Pipe Node]: init Variable success");


  return true;
}

/*###################################*/
/* Subscriber initialize function */
/*###################################*/
bool ConvertPipeNode::initSubscriber(){
  
//   traj_velo_subscriber_          = nh_.subscribe<geometry_msgs::Twist>("/wind_turbine_insp/traj_velo_ctrl", 2, &ConvertPipeNode::trajVeloCtrlCallback, this);
  

//   ROS_INFO_STREAM("Convert Pipe Node]: init Subscriber success");

  return true;
}


/*###################################*/
/* ServiceClient initialize function */
/*###################################*/
bool ConvertPipeNode::initServiceClient(){


//   ROS_INFO_STREAM("Convert Pipe Node]: init ServiceClient");

  return true;
}

/*###################################*/
/* ServiceServer initialize function */
/*###################################*/
bool ConvertPipeNode::initServiceServer(){
  
//   ROS_INFO_STREAM("Convert Pipe Node]: init ServiceServer");
  return true;
}

/*###################################*/
/*   Publisher initialize function   */
/*###################################*/
bool ConvertPipeNode::initPublisher(){
    // set_velo_publisher_                  = nh_.advertise<insp_msgs::VeloCmd>("/wind_turbine_insp/set_velo", 2);
    // wps_servo_publisher_                 = nh_.advertise<geometry_msgs::PoseStamped >("/wind_turbine_insp/set_waypoint_common", 2);
    // wps_traj_publisher_                  = nh_.advertise<geometry_msgs::PoseStamped >("/wind_turbine_insp/set_waypoint_minco", 1);
    

    // ROS_INFO_STREAM("Convert Pipe Node]:init Publisher success");

    return true;
}


// 把yaw转到0-360度
double normalize_yaw(double yaw) {
    // 将弧度转换为度数
    double yaw_deg = yaw * 180.0 / M_PI;

    // 正规化到 0 到 360 度
    if (yaw_deg < 0) {
        yaw_deg += 360;
    }
    return yaw_deg;
}


// 0.1.4 接收实际的odom
  void odom_callback(const nav_msgs::Odometry::ConstPtr& msgPtr) {
    odom_msg_ = *msgPtr;
    odom_received_ = true;

    rcv_odom_stamp = ros::Time::now();

    odom_p.x() = odom_msg_.pose.pose.position.x;
    odom_p.y() = odom_msg_.pose.pose.position.y;
    odom_p.z() = odom_msg_.pose.pose.position.z;

    odom_v.x() = odom_msg_.twist.twist.linear.x;
    odom_v.y() = odom_msg_.twist.twist.linear.y;
    odom_v.z() = odom_msg_.twist.twist.linear.z;
    
    // 获取四元数
    double qx = msgPtr->pose.pose.orientation.x;
    double qy = msgPtr->pose.pose.orientation.y;
    double qz = msgPtr->pose.pose.orientation.z;
    double qw = msgPtr->pose.pose.orientation.w;

    // 创建四元数
    Eigen::Quaterniond quaternion(qw, qx, qy, qz);
    
    // 将四元数转换为旋转矩阵
    Eigen::Matrix3d rotation_matrix = quaternion.toRotationMatrix();
    
    // 从旋转矩阵获取偏航角（yaw）
    double yaw = atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));
    
    // 将偏航角转换到 0 到 360 度的范围
    // odom_yaw_deg = normalize_yaw(yaw);

    // 如果用的是弧度，把上面这两行注释换成下面这个
    odom_yaw_rad = yaw;
  }


/*################################*/
/*    Timer initialize function   */
/*################################*/

bool ConvertPipeNode::initTimer(){
    int rate_convert_pipe_ = 3;  // 9.9.11   3  6
    // 原本计划是管道发送指令，但是轮退也用ros，这个留下来。但是保留直接发送轨迹的控制点的功能
    convert_msg_2_pipe_timer_  = nh_.createTimer(ros::Rate(rate_convert_pipe_), &ConvertPipeNode::convertMsg2PipeCallback, this);
    // 处理用户发过来的目标点
    user_input_global_points_timer_  = nh_.createTimer(ros::Rate(rate_convert_pipe_), &ConvertPipeNode::userInputGoalPointsCallback, this);
    int rate_pid_ctrl_ = 60;
    pid_ctrl_timer_  = nh_.createTimer(ros::Rate(rate_pid_ctrl_), &ConvertPipeNode::pidCtrlCallback, this);

    pid_ctrl_timer_yaw  = nh_.createTimer(ros::Rate(rate_pid_ctrl_), &ConvertPipeNode::pidCtrlCallback_yaw, this);
    force_stop = nh_.createTimer(ros::Rate(rate_pid_ctrl_), &ConvertPipeNode::stopCallback, this);

    ROS_INFO_STREAM("[Convert Pipe Node]: init timer success");

    return true;
}

/*###############################*/
/*  subscriber Callback function */
/*################################*/
// void ConvertPipeNode::trajVeloCtrlCallback(const geometry_msgs::Twist::ConstPtr& msg){
//     set_velo_.x = msg -> linear.x;
//     set_velo_.y = msg -> linear.y;
//     set_velo_.z = msg -> linear.z;
// }

/*###############################*/
/*      获取重规划还剩多少时间       */
/*################################*/
void last_traj_t_rest_callback(geometry_msgs::PoseStampedConstPtr pMsg){

    last_traj_t_rest_msg = *pMsg;
    
    last_traj_t_rest_global = last_traj_t_rest_msg.pose.position.x;
    // ROS_INFO("[convert pipe node]: last_traj_t_rest_global : %f", last_traj_t_rest_global);

}

/*###########################*/
/* 处理用户输入并填充队列的函数  */
/*##########################*/
void processInput() {
    // 检查输入的数量是否为3的倍数
    if (user_keyboard_input_points.size() % 3 != 0) {
        std::cerr << "输入的数字总数必须是3的倍数!" << std::endl;
        return;
    }

    // 将每三个数提取到队列中
    for (size_t i = 0; i < user_keyboard_input_points.size(); i += 3) {
        Eigen::Vector3d point(user_keyboard_input_points[i],
                               user_keyboard_input_points[i + 1],
                               user_keyboard_input_points[i + 2]);
        move_base_goal_queue_.push(point);
    }

    // 下面是验证代码，不要解开注释，不然无法运行！会弹出所有数据。验证了没问题
    // std::cout << "提取到的点：" << std::endl;
    // while (!move_base_goal_queue_.empty()) {
    //     Eigen::Vector3d point = move_base_goal_queue_.front();
    //     std::cout << "Point: [" << point(0) << ", " << point(1) << ", " << point(2) << "]" << std::endl;
    //     move_base_goal_queue_.pop();
    // }
}

/*###############################*/
/*      get the cmd callback     */
/*################################*/
void position_cmd_callback(quadrotor_msgs::PositionCommandConstPtr pMsg){

    cmd_msg = *pMsg;
    rcv_last_cmd_stamp = ros::Time::now();    
    trajectory_id_new = cmd_msg.trajectory_id;
    
    cmd_p(0) = cmd_msg.position.x;
    cmd_p(1) = cmd_msg.position.y;
    cmd_p(2) = cmd_msg.position.z;

    cmd_v(0) = cmd_msg.velocity.x;
    cmd_v(1) = cmd_msg.velocity.y;
    cmd_v(2) = cmd_msg.velocity.z;

    cmd_a(0) = cmd_msg.acceleration.x;
    cmd_a(1) = cmd_msg.acceleration.y;
    cmd_a(2) = cmd_msg.acceleration.z;

    cmd_j(0) = cmd_msg.jerk.x;
    cmd_j(1) = cmd_msg.jerk.y;
    cmd_j(2) = cmd_msg.jerk.z;
    
    cmd_yaw = cmd_msg.yaw;
    cmd_yaw_rate = cmd_msg.yaw_dot;
    trajectory_id_old = cmd_msg.trajectory_id;

    //9.9.10 把traj终点的时间延长一点
    
    // std::cout << "traj id old: " << trajectory_id_old << std::endl;
        // ROS_INFO("[convert node]: callback  receive postion cmd.");

}
 

// 0.1.2 接收轮腿消息，实际这里不适用
void wheel_leg_pos_cmd_callback(geometry_msgs::PointConstPtr pMsg){
    wheel_leg_pos_cmd_msg = *pMsg;
    wheel_leg_pos_x = wheel_leg_pos_cmd_msg.x;
    wheel_leg_pos_y = wheel_leg_pos_cmd_msg.y;
    wheel_leg_pos_z = wheel_leg_pos_cmd_msg.z;
}
void wheel_leg_yaw_cmd_callback(geometry_msgs::PointConstPtr pMsg){
    wheel_leg_yaw_cmd_msg = *pMsg;
    wheel_leg_yaw = wheel_leg_yaw_cmd_msg.x;
    wheel_leg_yaw_dot = wheel_leg_yaw_cmd_msg.y;
    wheel_leg_state_number = wheel_leg_yaw_cmd_msg.z;
}


/*###############################*/
/*    Timer Callback function    */
/*################################*/
void ConvertPipeNode::convertMsg2PipeCallback(const ros::TimerEvent& event){
    // set_velo_publisher_.publish(set_velo_);

    // init msg pipe
    msgid = msgget(MSGKEY, IPC_CREAT | 0666);
    if (-1 == msgid) {
        ROS_WARN("[Convert Pipe Node]: Message pipe get error!");
        return;
    }
    info_time_ = ros::Time::now();

    // msg init
    // 这边的控制指令会在更改目标点后自动更新
    pipeMessageBuffer commandMessage;
    commandMessage.mType = 1;
    double data[9] = {cmd_p(0),cmd_p(1),cmd_p(2),cmd_v(0),cmd_v(1),cmd_v(2),cmd_yaw,cmd_yaw_rate,state_number}; // 测试
    memcpy(commandMessage.data, data, sizeof(data));

        // 0.1.2 发布控制指令给轮腿    // 0.1.4 判断是否使用仿真控制器，不用的话就直接进入到下面的PID
    if(!is_use_simulation_control){
        geometry_msgs::Point pos_cmd;
        pos_cmd.x = cmd_p(0);
        pos_cmd.y = cmd_p(1);
        pos_cmd.z = cmd_p(2);
        wheel_leg_pos_cmd_pub.publish(pos_cmd); 
        geometry_msgs::Point yaw_cmd;
        yaw_cmd.x = cmd_yaw;
        yaw_cmd.y = cmd_yaw_rate;
        yaw_cmd.z = state_number;
        wheel_leg_yaw_cmd_pub.publish(yaw_cmd); 

    }
    
    if(readyToSendPipeMessage && msgid != -1)
    {
        // send msg into pipe
        int ret = msgsnd(msgid, &commandMessage, sizeof(commandMessage.data), 0);
        if ( ret == -1 ) {
            ROS_WARN("[Convert Pipe Node]: Message send failed!");
            return;
        } else {
            // 调试信息
            // std::cout << "Sent message to queue." << std::endl;
        }
    }

    return;
}

double err_yaw( double des_yaw, double odom_yaw)
{
  if(des_yaw - odom_yaw >= 3.14159)
    return (des_yaw - odom_yaw) - 2 * 3.14159;
  else if(des_yaw - odom_yaw <= -3.14159)
    return 2 * 3.14159 + (des_yaw - odom_yaw);
  else
    return (des_yaw - odom_yaw); 
}


/*###############################*/
/*     PID控制位置环和yaw环         */
/*################################*/
void ConvertPipeNode::pidCtrlCallback(const ros::TimerEvent& event){
    if(!pid_near && !stop_car){
         if(odom_received_){
        if (is_use_simulation_control)
        {
            double x_pos_error = cmd_p.x()-odom_p(0);
         //   std::cout<< "------------x_pos_error--------------------"<<cmd_p.x() << std::endl;
            double des_v_x = pid_ctrl_x.velCtrl(x_pos_error);
            double y_pos_error = cmd_p.y()-odom_p(1);
         //   std::cout<< "------------y_pos_error--------------------"<< cmd_p.y() << std::endl;
            Eigen::Vector2d err_posxy = Eigen::Vector2d(x_pos_error, y_pos_error);
            Eigen::Vector2d forward_dir = Eigen::Vector2d( cos(odom_yaw_rad) , sin(odom_yaw_rad) ); //当前机器人前进方向
            double pos_err = forward_dir.dot(err_posxy);
         //   std::cout<< "------------err_posxy--------------------"<< err_posxy << std::endl;
            double des_v_y = pid_ctrl_y.velCtrl(pos_err);
           // double des_v_y = pid_ctrl_y.velCtrl(y_pos_error);
            double z_pos_error = cmd_p.z()-odom_p(2);
            // 控制z速度没用 我直接复制过来了
            // double des_v_z = pid_ctrl_z.velCtrl(z_pos_error);
            // yaw的控制器，把z轴速度控制器给了控制yaw
            //double yaw_error = cmd_yaw - odom_yaw_rad; // TODO：deg还是rad需要
            double yaw_error = err_yaw(cmd_yaw,odom_yaw_rad);
            std::cout<< "------------cmd_yaw--------------------    "<< cmd_yaw << std::endl;
            std::cout<< "------------odom_yaw_rad--------------------    "<< odom_yaw_rad << std::endl;
          //  std::cout<< "------------yaw_error--------------------"<< yaw_error << std::endl;
            double des_yaw_dot = pid_ctrl_z.velCtrl(yaw_error);
         //   std::cout<< "------------des_yaw_dot-------------------   "<< des_yaw_dot << std::endl;
            // 发布到轮腿
            geometry_msgs::Point vel_cmd;
            vel_cmd.x = des_v_x;
            vel_cmd.y = des_v_y;
            vel_cmd.z = 0;
           // Eigen::Vector2d cmd_v = des_v_y; //机器人需要的前进速度
            double forward_speed =des_v_y;
            

            // 发布diablo话题
            cmd_dia.speed = forward_speed;
            cmd_dia.omega = des_yaw_dot;
            if (!stop_car){ // 如果没有停车
                ROS_ERROR("[convert pipr node]: !!!!!!!!!!!as normal");
                diablo_cmd_pub.publish(cmd_dia);  
            }
            

            wheel_leg_pos_cmd_pub.publish(vel_cmd); 
            geometry_msgs::Point yaw_cmd;
            yaw_cmd.x = cmd_yaw;
            yaw_cmd.y = des_yaw_dot;
            yaw_cmd.z = state_number; // 错误代码 先没用预留出来
            wheel_leg_yaw_cmd_pub.publish(yaw_cmd); 
        }
        
    } else {
        ROS_INFO("[convert pipr node]: don't recv odom msg!");
    }


    }
   
}

void ConvertPipeNode::pidCtrlCallback_yaw(const ros::TimerEvent& event){
    if(pid_near){
        if(odom_received_){
        if (is_use_simulation_control)
        {
            double x_pos_error = temp_goal_point.x()-odom_p(0);
            double y_pos_error = temp_goal_point.y()-odom_p(1);
            Eigen::Vector2d err_posxy = Eigen::Vector2d(x_pos_error, y_pos_error);
            Eigen::Vector2d forward_dir = Eigen::Vector2d( cos(odom_yaw_rad) , sin(odom_yaw_rad) ); //当前机器人前进方向
            double pos_err = forward_dir.dot(err_posxy);
            double des_v_y = pid_ctrl_y.velCtrl(pos_err);

            double yaw_error = err_yaw(yaw_goal_temp,odom_yaw_rad);
            double des_yaw_dot = pid_ctrl_yaw_near.velCtrl(yaw_error);
            // 发布到轮腿
            geometry_msgs::Point vel_cmd;
            vel_cmd.x = 0;
            vel_cmd.y = des_v_y;
            vel_cmd.z = 0;
           // Eigen::Vector2d cmd_v = des_v_y; //机器人需要的前进速度
            double forward_speed =des_v_y;
            

            // 发布diablo话题
            //cmd_dia.speed = forward_speed;
            cmd_dia.speed = 0;
            cmd_dia.omega = des_yaw_dot;
            diablo_cmd_pub.publish(cmd_dia);  
            ROS_ERROR("[convert pipr node]: !!!!!!!!!!!PID_______CONTROLLLLLLLLLLLLLL");

            wheel_leg_pos_cmd_pub.publish(vel_cmd); 
            geometry_msgs::Point yaw_cmd;
            yaw_cmd.x = cmd_yaw;
            yaw_cmd.y = des_yaw_dot;
            yaw_cmd.z = state_number; // 错误代码 先没用预留出来
            wheel_leg_yaw_cmd_pub.publish(yaw_cmd); 
            
        }
        
     } else {
         ROS_INFO("[convert pipr node]: don't recv odom msg!");
     }
    }
    else if(!stop_car) {
         ROS_INFO("[convert pipr node]: still far away!");

    }
}

// 9.9.10
// 当接近目标点（还没有在误差允许范围内），但是traj已经跑完了，且没有replan，则用最终的goal作为pid控制的标准
// 只调yaw

// void pid_near_goal(){
//         if(odom_received_){
//         if (is_use_simulation_control)
//         {
//             double x_pos_error = temp_goal_point.x()-odom_p(0);
//             double y_pos_error = temp_goal_point.y()-odom_p(1);
//             Eigen::Vector2d err_posxy = Eigen::Vector2d(x_pos_error, y_pos_error);
//             Eigen::Vector2d forward_dir = Eigen::Vector2d( cos(odom_yaw_rad) , sin(odom_yaw_rad) ); //当前机器人前进方向
//             double pos_err = forward_dir.dot(err_posxy);
//             double des_v_y = pid_ctrl_y.velCtrl(pos_err);

//             double yaw_error = err_yaw(yaw_goal_temp,odom_yaw_rad);
//             double des_yaw_dot = pid_ctrl_yaw_near.velCtrl(yaw_error);
//             // 发布到轮腿
//             geometry_msgs::Point vel_cmd;
//             vel_cmd.x = 0;
//             vel_cmd.y = des_v_y;
//             vel_cmd.z = 0;
//            // Eigen::Vector2d cmd_v = des_v_y; //机器人需要的前进速度
//             double forward_speed =des_v_y;
            

//             // 发布diablo话题
//            // cmd_dia.speed = forward_speed;
//            cmd_dia.speed = 0;
//             cmd_dia.omega = des_yaw_dot;
//             diablo_cmd_pub.publish(cmd_dia);  

//             wheel_leg_pos_cmd_pub.publish(vel_cmd); 
//             geometry_msgs::Point yaw_cmd;
//             yaw_cmd.x = cmd_yaw;
//             yaw_cmd.y = des_yaw_dot;
//             yaw_cmd.z = state_number; // 错误代码 先没用预留出来
//             wheel_leg_yaw_cmd_pub.publish(yaw_cmd); 
//         }
        
//     } else {
//         ROS_INFO("[convert pipr node]: don't recv odom msg!");
//     }
// }

// 9.9.10
void ConvertPipeNode::stopCallback(const ros::TimerEvent& event){
    if(stop_car){
        cmd_dia.speed = 0;
        cmd_dia.omega = 0;
        diablo_cmd_pub.publish(cmd_dia);  

        stop_msg_.x = 0;
        stop_msg_.y = 0;
        stop_msg_.z = 0;
        stop_pub.publish(stop_msg_);
        ROS_ERROR("[convert node] STOOOOOOOOOOOP the CAR.........");
    }
}

/*###############################*/
/*     处理用户输入的几个目标点      */
/*################################*/
int count_if_2 = 0;
int count_if_3 = 0;
int count_if_4 = 0;
    int task_choose_number = 0;
void ConvertPipeNode::userInputGoalPointsCallback(const ros::TimerEvent& event){   // 状态机更改

    // 0.1.1版本之后，直接开始重规划。如果需要用到到一个点停一下直接注释下面，把下下面的代码解开注释即可
    if (is_allow_to_start == 1)
    {
    //     if ((cmd_v(0) == 0) && (cmd_v(1) == 0) && (cmd_v(2) == 0) && (last_traj_t_rest_global < 0.001)) //如果到了目标点 0.001
    //     {
    //         ROS_INFO("[convert node]: reach first goal and last_traj_t_rest_global-------111: %f", last_traj_t_rest_global);
    //         sleep(2);
            

    //         //9.9.9 
    //             if(if_txtdot){
    //                 publishNextGoal_txt();
    //             }
    //             else publishNextGoal();
            
    //     } 
    //     if((cmd_v(0) != 0) && (cmd_v(1) != 0) && (cmd_v(2) != 0))  //如果没有到达目标点
    //     {
    //         // 0.1.4实际1.5偏大 改成1.0重规划
    //         if ((last_traj_t_rest_global < 1.0) ) //1.0  但是快到了
    //         {
    //             ROS_INFO("[convert node]: last_traj_t_rest_global-------222:%f",last_traj_t_rest_global);
    //             if ((count_if_3 < 1))    // 且之前没有改变发布下一个状态 count_if_3=0
    //             {
    //                 //9.9.9
    //                 if(if_txtdot){
    //                     publishNextGoal_txt();
    //                 }
    //                 else publishNextGoal();
    //                 ROS_INFO("[convert node]: publishNextGoal1--------33333:%d",count_if_3);
    //                 count_if_3++;
    //             }
    //         } else   // 还没有快到 
    //         {
    //                 ROS_INFO("[convert node]publishNextGoal2:---------4444:%d",count_if_3);
    //             count_if_3 = 0 ;
    //         }
            
    //     }
    // }
        //-------------------------------------------------------------------------------------------------------- 9.9.10
        // 拿到err
        // double x_pos_error = cmd_p.x()-odom_p(0);
        double x_pos_error = temp_goal_point.x()-odom_p(0);
        double des_v_x = pid_ctrl_x.velCtrl(x_pos_error);
        // double y_pos_error = cmd_p.y()-odom_p(1);
        double y_pos_error = temp_goal_point.y()-odom_p(1);
        Eigen::Vector2d err_posxy = Eigen::Vector2d(x_pos_error, y_pos_error);
        Eigen::Vector2d forward_dir = Eigen::Vector2d( cos(odom_yaw_rad) , sin(odom_yaw_rad) ); //当前机器人前进方向
        double pos_err = forward_dir.dot(err_posxy);
        double yaw_error = err_yaw(yaw_goal_temp,odom_yaw_rad);

        ROS_WARN("[convert node] pos_err: %f   yaw_error: %f ", pos_err , yaw_error);

        if ((pos_err < 0.2 ) && (std::fabs(yaw_error) < 0.05) ) // 当到了目标点附近允许范围内  0.1  0.05
        {
   //         ROS_INFO("[convert node]: reach goal: %d", temp_goal);
            ROS_INFO("[convert node]: pos_err: %f   yaw_error: %f ", pos_err , yaw_error);
            ROS_WARN("[convert node] reach goal...");
            pid_near = false;  
            ROS_WARN("[convert node] near pid false....................................");
            stop_car = true;
            ROS_WARN("[convert node] stop_car..........................................");
            sleep(2);                
           //9.9.9 
           if (!pos_queue.empty()){ // 如果到达的不是最后一个点，即原点
            if(if_txtdot){
                    publishNextGoal_txt();
                }
                else publishNextGoal();
            ROS_ERROR("[convert node] publishNextGoal...");
         //   sleep(2.0);//1.5
            stop_car = false;

           }   
           
        } 
        else if ((pos_err < 0.2) && (std::fabs(yaw_error) < 1.0)) { // 当接近目标点但还没有到达误差范围内，用pid调   0.35 1.0
            // if((cmd_v(0) == 0) && (cmd_v(1) == 0) && (cmd_v(2) == 0) && (last_traj_t_rest_global < 0.001)){
            //     ROS_INFO("[convert node]: near global: %d ",temp_goal);
            // }else{
                ROS_WARN("[convert node]: near global: %d , NOW PID_Control",temp_goal);
                ROS_INFO("[convert node]: pos_err: %f   yaw_error: %f ", pos_err , yaw_error);
                // for(int i=0; i<20; i++){
                //     pid_near_goal();
                //     }
                pid_near = true;
            // }

        }
    }
        

    return;
}


/*####################*/
/*   ctrl^c 终止函数   */
/*####################*/
void  INThandler(int sig)
{
    std::cout << "[Convert Pipe Node]: 关闭程序服务，退出程序..." << std::endl;
    readyToSendPipeMessage = false;
    // 删除消息队列
    if (msgctl(msgid, IPC_RMID, 0) == -1) {
        ROS_WARN("[Convert Pipe Node]: Delete message.");
    }
    if (!move_base_goal_queue_.empty()) {
        move_base_goal_queue_.pop();
    }
    ros::shutdown(); // 关闭 ROS 节点
    // 推出程序
    exit(sig);
}

//9.9.9 TXT
void  INThandler_txt(int sig){
    std::cout << "[Convert Pipe Node]: 关闭程序服务，退出程序..." << std::endl;
    readyToSendPipeMessage = false;
    // 删除消息队列
    if (msgctl(msgid, IPC_RMID, 0) == -1) {
        ROS_WARN("[Convert Pipe Node]: Delete message.");
    }
    if (!pos_queue.empty()) {
        pos_queue.pop();
        quat_queue.pop();
    }
    ros::shutdown(); // 关闭 ROS 节点
    // 推出程序
    exit(sig);
}
//  9.9.9 read TXT
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
    while (getline(file, line)) {
        line_num++;
        std::stringstream ss(line);
        std::string item;
        std::vector<double> row_pos_ori; // 用于存储单行数据的容器
        // 分割每一行的数据
        while (getline(ss, item, ',')) {
            double temp;
            std::istringstream iss(item);
            iss >> temp;
            row_pos_ori.push_back(temp);
        }
        pos_queue.push(Eigen::Vector3d(row_pos_ori[1], row_pos_ori[2], row_pos_ori[3]));
        quat_queue.push(Eigen::Vector4d(row_pos_ori[4], row_pos_ori[5], row_pos_ori[6],row_pos_ori[7]));
        std::cout<< row_pos_ori[1] << row_pos_ori[2] << row_pos_ori[3] << std::endl;
        row_pos_ori.clear();
    }
    
    // 把原点push进去
    pos_queue.push(Eigen::Vector3d(0, 0, 0.5));
    quat_queue.push(Eigen::Vector4d(1, 0, 0,0));

    // 关闭文件
    file.close();
    std::cout<< "close file !!" << std::endl;  
    goal_num = line_num;
    temp_goal = 0;


    std::cout << "[Convert Pipe Node]: -----接收的目标点个数为：" << goal_num << std::endl;

    
}


/*#################*/
/*                 */
/*  Main Function  */
/*                 */
/*#################*/
int main(int argc, char** argv){
    // init ros node
    ros::init(argc, argv, "convert_pipe_node");
    ros::NodeHandle nh("~");
    ConvertPipeNode convert_pipe_node(nh);
    ROS_INFO_STREAM("[Convert Pipe Node]: Convert Pipe Node is OK!");
    // timer thread
    ros::MultiThreadedSpinner spinner(2);
    readyToSendPipeMessage = true;
 
	signal(SIGINT, INThandler);
    // 读取用户发布的点，以队列形式储存（一个个输入或者修改）
    // 轮退的z轴都设定为1.5m
// 9.9.9
    nh.getParam("if_txtdot", if_txtdot);
    if(if_txtdot){
        ReadtxtGoal();
    }
    else{
        nh.getParam("p0x", p0x);
    nh.getParam("p1x", p1x);
    nh.getParam("p2x", p2x);
    nh.getParam("p3x", p3x);
    nh.getParam("p4x", p4x);
    nh.getParam("p0y", p0y);
    nh.getParam("p1y", p1y);
    nh.getParam("p2y", p2y);
    nh.getParam("p3y", p3y);
    nh.getParam("p4y", p4y);
    nh.getParam("p0z", p0z);
    nh.getParam("p1z", p1z);
    nh.getParam("p2z", p2z);
    nh.getParam("p3z", p3z);
    nh.getParam("p4z", p4z);
    // 组合
    // 将参数组合成 Eigen::Vector3d 对象并放入队列
    move_base_goal_queue_.push(Eigen::Vector3d(p0x, p0y, p0z));
    move_base_goal_queue_.push(Eigen::Vector3d(p1x, p1y, p1z));
    move_base_goal_queue_.push(Eigen::Vector3d(p2x, p2y, p2z));
    move_base_goal_queue_.push(Eigen::Vector3d(p3x, p3y, p3z));
    move_base_goal_queue_.push(Eigen::Vector3d(p4x, p4y, p4z));
    }
    


        // 0.1.4 读取PID系列参数
    nh.getParam("px4ctrl_kp0", kp0);
    nh.getParam("px4ctrl_kp1", kp1);
    nh.getParam("px4ctrl_kp2", kp2);

    nh.getParam("px4ctrl_ki0", ki0);
    nh.getParam("px4ctrl_ki1", ki1);
    nh.getParam("px4ctrl_ki2", ki2);

    nh.getParam("px4ctrl_kd0", kd0);
    nh.getParam("px4ctrl_kd1", kd1);
    nh.getParam("px4ctrl_kd2", kd2);

    nh.getParam("px4ctrl_kv0", kv0);
    nh.getParam("px4ctrl_kv1", kv1);
    nh.getParam("px4ctrl_kv2", kv2);
    nh.getParam("ctrl_frequency",ctrl_frequency);
    nh.getParam("is_use_simulation_control",is_use_simulation_control);
    nh.getParam("max_pos_err",max_pos_err);

    nh.getParam("v_max_x", v_max_x);
    nh.getParam("v_max_y", v_max_y);
    nh.getParam("v_max_z", v_max_z);

    nh.getParam("a_max_x", a_max_x);
    nh.getParam("a_max_y", a_max_y);
    nh.getParam("a_max_z", a_max_z);

    nh.getParam("pid_yaw_kp", pid_yaw_kp);
    nh.getParam("pid_yaw_ki", pid_yaw_ki);
    nh.getParam("pid_yaw_kd", pid_yaw_kd);
    nh.getParam("pid_yaw_kv", pid_yaw_kv);
    nh.getParam("vmax_yaw", vmax_yaw);
    nh.getParam("amax_yaw", amax_yaw);


    // pid controller initialization
    pid_ctrl_x = px4_ctrl::PIDctrl(1.0 / ctrl_frequency);
    pid_ctrl_y = px4_ctrl::PIDctrl(1.0 / ctrl_frequency); 
    pid_ctrl_z = px4_ctrl::PIDctrl(1.0 / ctrl_frequency);
    pid_ctrl_yaw_near = px4_ctrl::PIDctrl(1.0 / ctrl_frequency); //9.9.10

    // x轴速度控制器
    pid_ctrl_x.setParams(kp0,ki0,kd0,v_max_x,a_max_x);
    // y轴速度控制器
    pid_ctrl_y.setParams(kp1,ki1,kd1,v_max_y,a_max_y);
    // 名字叫z轴控制器，实际是yaw的控制器
    pid_ctrl_z.setParams(kp2,ki2,kd2,v_max_z,a_max_z);

    // near_pid 的参数
    pid_ctrl_yaw_near.setParams(pid_yaw_kp,pid_yaw_ki,pid_yaw_kd,vmax_yaw,amax_yaw);


    
    //创建订阅者，订阅话题postion_cmd
    position_cmd_subscriber_ = nh.subscribe<quadrotor_msgs::PositionCommand>("/target/position_cmd",10,position_cmd_callback);
    last_traj_t_rest_subscriber_ = nh.subscribe<geometry_msgs::PoseStamped>("/target/last_traj_t_rest_topic",10,last_traj_t_rest_callback);

        // 0.1.4接收实际的odom话题
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/new_odom", 10, odom_callback);


    // 创建发布者，发布/move_base_simple/goal(triger)话题
    triger_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    // 错开fake——targetlaunch的时间
    sleep(3);
    // main中发布第一个目标点
    std::cout << "p0 x y z : " << p0x << "  " << p0y << " " << p0z << std::endl;

    //9.9.9 TXT
    if(if_txtdot){
        publishFirstGoal_txt();
    }
    else publishFirstGoal();
    // is_allow_to_start = 1;

    // 0.1.1重规划启动
    // std::cin >> is_allow_to_start;

        // 0.1.2
    wheel_leg_pos_cmd_pub = nh.advertise<geometry_msgs::Point>("/wheel_leg/pos_cmd", 10);
    wheel_leg_yaw_cmd_pub = nh.advertise<geometry_msgs::Point>("/wheel_leg/yaw_cmd", 10);
    diablo_cmd_pub     = nh.advertise<motion_msgs::Diablo_Ctrl>("/target/control_ugv/cmd_dia", 50);
    stop_pub      = nh.advertise<geometry_msgs::Point>("/stop_msg", 10);


    
    wheel_leg_pos_cmd_subscriber = nh.subscribe<geometry_msgs::Point>("/wheel_leg/pos_cmd",10,wheel_leg_pos_cmd_callback);
    wheel_leg_yaw_cmd_subscriber = nh.subscribe<geometry_msgs::Point>("/wheel_leg/yaw_cmd",10,wheel_leg_yaw_cmd_callback);

    // 0.1.5
    //std::cout << "请输入一系列的double类型的数(总数为3的倍数),用空格分隔，按回车结束：" << std::endl;
    // 调用处理函数 会报错，放到线程里面去
    //processInput();


    spinner.spin();

    
    return 0;
}