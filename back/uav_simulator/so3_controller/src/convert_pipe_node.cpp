#include "so3_controller/convert_pipe_node.h"
#include "motion_msgs/Diablo_Ctrl.h"
#include "std_msgs/Bool.h"
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
double last_traj_t_rest_global;
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

ros::Publisher stop_pub;


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


double ctrl_frequency;// 控制频率 
double max_pos_err; // 最大位置误差，限幅

// 9.9.9
bool if_txtdot; // 是否打点
// 实例化PID控制器
px4_ctrl::PIDctrl pid_ctrl_x;
px4_ctrl::PIDctrl pid_ctrl_y;
px4_ctrl::PIDctrl pid_ctrl_z;
// 实际的odom
ros::Subscriber odom_sub_;
ros::Subscriber stop_sub_from_plan;
nav_msgs::Odometry stop_msg_from_plan;
nav_msgs::Odometry odom_msg_;
std::atomic_bool odom_received_ = ATOMIC_VAR_INIT(false);
ros::Time rcv_odom_stamp; // TODO：减去当前时间
// 当前实际的位置和速度
Eigen::Vector3d odom_p;
Eigen::Vector3d odom_v;
double odom_yaw_deg;
double odom_yaw_rad;
double odom_yaw_dot;


void stopCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data)
    {
        stop_car = true;
        ROS_INFO("Received stop true");
    }
    else
    {
        stop_car = false;
        ROS_INFO("Received stop false");
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

    odom_yaw_rad = yaw;
  }


/*################################*/
/*    Timer initialize function   */
/*################################*/

bool ConvertPipeNode::initTimer(){
    int rate_convert_pipe_ = 3;  // 9.9.11   3  6  11.29 3
    // 处理用户发过来的目标点
    int rate_pid_ctrl_ = 60;// 60 80
    pid_ctrl_timer_  = nh_.createTimer(ros::Rate(rate_pid_ctrl_), &ConvertPipeNode::pidCtrlCallback, this);

    ROS_INFO_STREAM("[Convert Pipe Node]: init timer success");

    return true;
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

    if(odom_received_){
            double x_pos_error = cmd_p.x()-odom_p(0);
         //   std::cout<< "------------x_pos_error--------------------"<<cmd_p.x() << std::endl;
         //   double des_v_x = pid_ctrl_x.velCtrl(x_pos_error);
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
        //     std::cout<< "------------cmd_yaw--------------------    "<< cmd_yaw << std::endl;
        //     std::cout<< "------------odom_yaw_rad--------------------    "<< odom_yaw_rad << std::endl;
        //    std::cout<< "------------yaw_error--------------------"<< yaw_error << std::endl;
            double des_yaw_dot = pid_ctrl_z.velCtrl(yaw_error);
         //   std::cout<< "------------des_yaw_dot-------------------   "<< des_yaw_dot << std::endl;
            // 发布到轮腿
            double forward_speed =des_v_y;
            

            // 发布diablo话题
            
            if (!stop_car){ // 如果没有停车
                //ROS_ERROR("[convert pipr node]: !!!!!!!!!!!as normal");
                cmd_dia.speed = forward_speed;
                cmd_dia.omega = des_yaw_dot;           
            }
            else {
                cmd_dia.speed = 0;
                cmd_dia.omega = 0;
            }
            diablo_cmd_pub.publish(cmd_dia);        
    } else {
        ROS_INFO("[convert pipr node]: don't recv odom msg!");
    }
      
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
 
	signal(SIGINT, INThandler_txt); //------------- 11.21
    // 读取用户发布的点，以队列形式储存（一个个输入或者修改）
    // 轮退的z轴都设定为1.5m
// 9.9.9
    nh.getParam("if_txtdot", if_txtdot);
    if(if_txtdot){
        // ReadtxtGoal(); 
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
    nh.getParam("max_pos_err",max_pos_err);

    nh.getParam("v_max_x", v_max_x);
    nh.getParam("v_max_y", v_max_y);
    nh.getParam("v_max_z", v_max_z);

    nh.getParam("a_max_x", a_max_x);
    nh.getParam("a_max_y", a_max_y);
    nh.getParam("a_max_z", a_max_z);



    // pid controller initialization
    pid_ctrl_x = px4_ctrl::PIDctrl(1.0 / ctrl_frequency);
    pid_ctrl_y = px4_ctrl::PIDctrl(1.0 / ctrl_frequency); 
    pid_ctrl_z = px4_ctrl::PIDctrl(1.0 / ctrl_frequency);

    // x轴速度控制器
    pid_ctrl_x.setParams(kp0,ki0,kd0,v_max_x,a_max_x);
    // y轴速度控制器
    pid_ctrl_y.setParams(kp1,ki1,kd1,v_max_y,a_max_y);
    // 名字叫z轴控制器，实际是yaw的控制器
    pid_ctrl_z.setParams(kp2,ki2,kd2,v_max_z,a_max_z);

    
    //创建订阅者，订阅话题postion_cmd
    position_cmd_subscriber_ = nh.subscribe<quadrotor_msgs::PositionCommand>("/target/planning/position_cmd",10,position_cmd_callback);

        // 0.1.4接收实际的odom话题
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/new_odom", 10, odom_callback);
    stop_sub_from_plan = nh.subscribe<std_msgs::Bool>("/stop_msg", 10, stopCallback);


    // 创建发布者，发布/move_base_simple/goal(triger)话题
    triger_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    // 错开fake——targetlaunch的时间
    sleep(3);

    //9.9.9 TXT
    if(if_txtdot){
        // publishFirstGoal_txt();
    };
 
        // 0.1.2
    diablo_cmd_pub     = nh.advertise<motion_msgs::Diablo_Ctrl>("/target/control_ugv/cmd_dia", 50);

  
    spinner.spin();

    
    return 0;
}