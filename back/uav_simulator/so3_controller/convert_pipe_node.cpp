#include "wind_turbine_insp/convert_pipe_node.h"
using namespace class_convert_pipe;


/*#####################*/
/* Message struct      */
/* 消息类型列表：        */
/* 1：xyz速度，wxyz姿态  */
/*#####################*/
int msgid;
bool readyToSendPipeMessage = false; // 当标志位为true运行管道通信
#define MSGKEY 1001
#define MSGTYPE 1
struct pipeMessageBuffer{
    long mType; // msg type
    double data[7]; // calocity xyz, quat wxyz
};


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

  printf("Loading Params Finish\n");
  printf("---------------------\n");

  

  ROS_INFO_STREAM("Convert Pipe Node]: init Variable success");


  return true;
}

/*###################################*/
/* Subscriber initialize function */
/*###################################*/
bool ConvertPipeNode::initSubscriber(){
  
  traj_velo_subscriber_          = nh_.subscribe<geometry_msgs::Twist>("/wind_turbine_insp/traj_velo_ctrl", 2, &ConvertPipeNode::trajVeloCtrlCallback, this);

  ROS_INFO_STREAM("Convert Pipe Node]: init Subscriber success");

  return true;
}


/*###################################*/
/* ServiceClient initialize function */
/*###################################*/
bool ConvertPipeNode::initServiceClient(){


  ROS_INFO_STREAM("Convert Pipe Node]: init ServiceClient");

  return true;
}

/*###################################*/
/* ServiceServer initialize function */
/*###################################*/
bool ConvertPipeNode::initServiceServer(){
  
  ROS_INFO_STREAM("Convert Pipe Node]: init ServiceServer");
}

/*###################################*/
/*   Publisher initialize function   */
/*###################################*/
bool ConvertPipeNode::initPublisher(){
    set_velo_publisher_                  = nh_.advertise<insp_msgs::VeloCmd>("/wind_turbine_insp/set_velo", 2);
    wps_servo_publisher_                 = nh_.advertise<geometry_msgs::PoseStamped >("/wind_turbine_insp/set_waypoint_common", 2);
    wps_traj_publisher_                  = nh_.advertise<geometry_msgs::PoseStamped >("/wind_turbine_insp/set_waypoint_minco", 1);
    

    ROS_INFO_STREAM("Convert Pipe Node]:init Publisher success");

    return true;
}

/*################################*/
/*    Timer initialize function   */
/*################################*/

bool ConvertPipeNode::initTimer(){
    int rate_convert_pipe_ = 20;
    convert_msg_2_pipe_timer_  = nh_.createTimer(ros::Rate(rate_convert_pipe_), &ConvertPipeNode::convertMsg2PipeCallback, this);
    ROS_INFO_STREAM("[Convert Pipe Node]: init timer success");

    return true;
}

/*###############################*/
/*  subscriber Callback function */
/*################################*/
void ConvertPipeNode::trajVeloCtrlCallback(const geometry_msgs::Twist::ConstPtr& msg){
    set_velo_.x = msg -> linear.x;
    set_velo_.y = msg -> linear.y;
    set_velo_.z = msg -> linear.z;
}


/*###############################*/
/*    Timer Callback function    */
/*################################*/
void ConvertPipeNode::convertMsg2PipeCallback(const ros::TimerEvent& event){
    set_velo_publisher_.publish(set_velo_);

    // init msg pipe
    msgid = msgget(MSGKEY, IPC_CREAT | 0666);
    if (-1 == msgid) {
        ROS_WARN("[Convert Pipe Node]: Message pipe get error!");
        return;
    }
    info_time_ = ros::Time::now();

    // msg init
    pipeMessageBuffer commandMessage;
    commandMessage.mType = 1;
    double data[7] = {1.0,2.0,3.0,4.0,5.0,6.0,7.0}; // 测试
    memcpy(commandMessage.data, data, sizeof(data));

    
    if(readyToSendPipeMessage && msgid != -1)
    {
        // send msg into pipe
        int ret = msgsnd(msgid, &commandMessage, sizeof(commandMessage.data), 0);
        if ( ret == -1 ) {
            ROS_WARN("[Convert Pipe Node]: Message send failed!");
            return;
        } else {
            // 调试信息
            std::cout << "Sent message to queue." << std::endl;
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
    ros::NodeHandle nh;
    ConvertPipeNode convert_pipe_node(nh);
    ROS_INFO_STREAM("[Convert Pipe Node]: Convert Pipe Node is OK!");
    // timer thread
    ros::MultiThreadedSpinner spinner(1);
    readyToSendPipeMessage = true;

	signal(SIGINT, INThandler);

    spinner.spin();

    
    return 0;
}