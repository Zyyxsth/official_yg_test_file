#include "so3_controller/convert_pipe_node.h"

// 0.1.2 适配轮腿
#include <geometry_msgs/Point.h>

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
double wheel_leg_pos_x = 0.0;
double wheel_leg_pos_y = 0.0;
double wheel_leg_pos_z = 0.0;
double wheel_leg_yaw = 0.0;
double wheel_leg_yaw_dot = 0.0;
double wheel_leg_state_number = 0.0;
geometry_msgs::Point wheel_leg_pos_cmd_msg;
geometry_msgs::Point wheel_leg_yaw_cmd_msg;


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
    sleep(2);
    if (!move_base_goal_queue_.empty()) {
        move_base_goal_queue_.pop();
        ROS_INFO("[convert pirp node]: reach the first goal height.");
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

/*################################*/
/*    Timer initialize function   */
/*################################*/

bool ConvertPipeNode::initTimer(){
    int rate_convert_pipe_ = 3;
    convert_msg_2_pipe_timer_  = nh_.createTimer(ros::Rate(rate_convert_pipe_), &ConvertPipeNode::convertMsg2PipeCallback, this);
    user_input_global_points_timer_  = nh_.createTimer(ros::Rate(rate_convert_pipe_), &ConvertPipeNode::userInputGoalPointsCallback, this);
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

    // 0.1.2 发布控制指令给轮腿
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

/*###############################*/
/*     处理用户输入的几个目标点      */
/*################################*/
int count_if_2 = 0;
int count_if_3 = 0;
int count_if_4 = 0;
    int task_choose_number = 0;
void ConvertPipeNode::userInputGoalPointsCallback(const ros::TimerEvent& event){

    // 0.1.1版本之后，直接开始重规划
    if (is_allow_to_start == 1)
    {
        if ((cmd_v(0) == 0) && (cmd_v(1) == 0) && (cmd_v(2) == 0) && (last_traj_t_rest_global < 0.001)) 
        {
            ROS_INFO("[convert node]: reach first goal and last_traj_t_rest_global: %f", last_traj_t_rest_global);
            publishNextGoal();
        } 
        if((cmd_v(0) != 0) && (cmd_v(1) != 0) && (cmd_v(2) != 0)) 
        {
            if ((last_traj_t_rest_global < 1.5) )
            {
                ROS_INFO("[convert node]: last_traj_t_rest_global:%f",last_traj_t_rest_global);
                if ((count_if_3 < 1))
                {
                    publishNextGoal();
                    ROS_INFO("[convert node]: publishNextGoal1:%d",count_if_3);
                    count_if_3++;
                }
            } else
            {
                    ROS_INFO("[convert node]: publishNextGoal2:%d",count_if_3);
                count_if_3 = 0 ;
            }
            
        }
        
    }
    
    

    // // 0.1.1版本之前，需要每个规划点停一下再开始下一次规划。要用的话注释掉就好
    // // 第一个点不会被这里面的逻辑影响
    // // 到达目标点
    // // 使用计时器判断是否还能接收到position_cmd的消息
    // double update_dt = (ros::Time::now() - rcv_last_cmd_stamp).toSec();
    // if (update_dt > 2.0)
    // {
    //     ROS_INFO("[convert node]: don't recv cmd data.");
    //     is_reach_one_goal_ = true;
    // } else { // 当一直在接收，且速度不为0，说明没有到达目标点。不允许中途更换
    //     if ((cmd_v(0) != 0) && (cmd_v(1) != 0) && (cmd_v(2) != 0))
    //     {
    //         is_allow_send_next_point = false;
    //         is_reach_one_goal_ = false;
    //     }
    //     // 当速度为0，且标志位“is_allow_send_next_point”为0，不允许发送下一个点
    //     if ((cmd_v(0) == 0) && (cmd_v(1) == 0) && (cmd_v(2) == 0) && (is_allow_send_next_point == false))
    //     { // 一直在接收消息且速度为0，说明已经到达第一个点。
    //         is_reach_one_goal_ = true;
    //         count_if_2 = 0;
    //         count_if_3 = 0;
    //         count_if_4 = 0;
    //         task_choose_number = 0;
    //         ROS_INFO("[convert node]: recv cmd data, reach the goal.");
    //     }
    //     // 当速度为0，且标志位“is_allow_send_next_point”为1，准备发送下一个点
    //     if ((cmd_v(0) == 0) && (cmd_v(1) == 0) && (cmd_v(2) == 0) && (is_allow_send_next_point == true) && (count_if_4 < 1)) // 当到达目标点之后，各种状态清0
    //     { 
    //         // 模拟用户万一想要停在某个点，模拟一下，随时可以去掉
    //         // sleep(0.3);
    //         publishNextGoal();
    //         count_if_4++; // 只能执行一次，不然astar out memory
    //         ROS_INFO("[convert node]: go to the next goal.");
    //     }
        
    // }
    // // 设计为当你到达下一个
    // if (is_reach_one_goal_  && (count_if_2 < 1))
    // {
    //     if (task_choose_number == 0)
    //     {
    //         ROS_INFO("[convert node]: success one of the goal, please enter next task.");
    //         ROS_INFO("[convert node]: 1. go next point. 2. stop.");
    //         // std::cin >> task_choose_number ;
    //         task_choose_number =1;
    //         // 模拟用户万一想要停在某个点，模拟一下，随时可以去掉
    //         // sleep(1);
    //     }
    //     if (task_choose_number == 1)
    //     {
    //         ROS_INFO("[convert node]: allow to go to the next goal.");
    //         is_allow_send_next_point = true;
    //         count_if_2++;
    //     }
        
    // }
    // // 当允许发送下一个点，弹出第一个点，跟踪新的点
    // if (is_allow_send_next_point && (count_if_3 < 1))
    // {
    //     is_reach_one_goal_ = false;
    //     ROS_INFO("[convert node]: ready to send the next goal.");
    //     count_if_3++;
    // } // else 保持当前高度交给规划器
    
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
    //创建订阅者，订阅话题postion_cmd
    position_cmd_subscriber_ = nh.subscribe<quadrotor_msgs::PositionCommand>("/target/position_cmd",10,position_cmd_callback);
    last_traj_t_rest_subscriber_ = nh.subscribe<geometry_msgs::PoseStamped>("/target/last_traj_t_rest_topic",10,last_traj_t_rest_callback);

    // 创建发布者，发布/move_base_simple/goal(triger)话题
    triger_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    // 错开fake——targetlaunch的时间
    sleep(3);
    // main中发布第一个目标点
    std::cout << "p0 x y z : " << p0x << "  " << p0y << " " << p0z << std::endl;
    publishFirstGoal();

    // 0.1.2
    wheel_leg_pos_cmd_pub = nh.advertise<geometry_msgs::Point>("/wheel_leg/pos_cmd", 10);
    wheel_leg_yaw_cmd_pub = nh.advertise<geometry_msgs::Point>("/wheel_leg/yaw_cmd", 10);
    wheel_leg_pos_cmd_subscriber = nh.subscribe<geometry_msgs::Point>("/wheel_leg/pos_cmd",10,wheel_leg_pos_cmd_callback);
    wheel_leg_yaw_cmd_subscriber = nh.subscribe<geometry_msgs::Point>("/wheel_leg/yaw_cmd",10,wheel_leg_yaw_cmd_callback);


    spinner.spin();

    
    return 0;
}