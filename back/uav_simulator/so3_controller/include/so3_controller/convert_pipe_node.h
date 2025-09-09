#ifndef __CONVERT_PIPE_NODE_H__
#define __CONVERT_PIPE_NODE_H__

#include <ros/ros.h>
#include <iostream>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <atomic>
#include <queue>
/* vins */
// #include <visualization/visualization.hpp>
/* tf_msgs */
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
/* std_msgs */
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
/* sensor_msgs */
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
/* nav_msgs */
#include <nav_msgs/Odometry.h>
/* geometry_msgs */
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/QuaternionStamped.h>
/* mavros_msgs */
// #include <mavros_msgs/SetMode.h>
// #include <mavros_msgs/CommandBool.h>
// #include <mavros_msgs/State.h>
// #include <mavros_msgs/MountControl.h>
/* command msgs */
// #include <insp_msgs/VeloCmd.h>
/* message pipe */
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <string.h>
#include <signal.h>
/* quadrotor msg */
#include "quadrotor_msgs/PositionCommand.h"
// 0.1.4 PID控制
#include "so3_controller/pid_ctrl.h"


namespace class_convert_pipe{

  #define PI (double)3.14159265358979323846
  #define R_EARTH (double)6378137.0

  #define DEG2RAD(DEG) ((DEG) * (PI) / (180.0))
  #define RAD2DEG(RAD) ((RAD) * (180.0) / (PI))


  class ConvertPipeNode{

    public:

        ConvertPipeNode(ros::NodeHandle nh);
        ~ConvertPipeNode();
        
        bool initVariable();

        bool initServiceServer();
        bool initServiceClient();

        bool initPublisher();
        bool initSubscriber();

        bool initTimer();
        


    protected:
        /* /////--------------------- Services ---------------------///// */
        /* ---------------------- states & control server ---------------------- */
        // ros::ServiceServer set_stage_mode_server_;
        // ros::ServiceServer set_flight_task_server_;
        // ros::ServiceServer emergency_terminate_server_;
        // ros::Publisher node_status_publisher_;
        // ros::Publisher debug_master_publisher_;

        /* ---------------------- mavros clients ---------------------- */  
        // ros::ServiceClient mavros_arming_client_;
        // ros::ServiceClient mavros_set_mode_client_;
        

        /* /////--------------------- Publisher ---------------------///// */
        /* ---------------------- states & control publisher ---------------------- */
        // ros::Publisher stage_mode_publisher_;
        // ros::Publisher emergency_terminate_publisher_;
        // ros::Publisher set_velo_publisher_;
        // ros::Publisher wps_servo_publisher_;
        // ros::Publisher wps_traj_publisher_;

        /* ---------------------- mavros publisher ---------------------- */
        // ros::Publisher mavros_set_target_velo_publisher_;

        /* ---------------------- odom publisher ---------------------- */
        // ros::Publisher local_odom_publisher_;
        // ros::Publisher debug_drone_odom_enu_pub;
        // ros::Publisher debug_drone_odom_flu_pub;



        /* /////--------------------- Subscriber ---------------------///// */
        /* ---------------------- mavros subscriber ---------------------- */  
        // ros::Subscriber mavros_state_subscriber_;
        // ros::Subscriber mavros_local_odom_subscriber_;
        // ros::Subscriber set_velo_subscriber_;
        /* ---------------------- states & control subscriber ---------------------- */
        // ros::Subscriber traj_velo_subscriber_;
        /* position_cmd command */
        // ros::Subscriber position_cmd_subscriber_;


        /* /////--------------------- Timer ---------------------///// */
        ros::Timer convert_msg_2_pipe_timer_;
        ros::Timer user_input_global_points_timer_;
        ros::Timer pid_ctrl_timer_;
        ros::Timer pid_ctrl_timer_yaw;
        ros::Timer force_stop;

    protected:

        /* /////--------------------- Callback Function ---------------------///// */
        /* ----------- ServiceServer callback function ------------ */
        // bool setStageModeCallback(insp_msgs::SetStageMode::Request& request, insp_msgs::SetStageMode::Response& response);
        // bool setFlightTaskCallback(insp_msgs::FlightTaskControl::Request& request, insp_msgs::FlightTaskControl::Response& response);
        // bool emergencyTerminateCallback(insp_msgs::EmergencyTerminate::Request& request, insp_msgs::EmergencyTerminate::Response& response);
        
        /* ----------- Subscriber callback function ----------- */
        // void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg);
        // void mavrosLocalOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        // void setVeloCallback(const insp_msgs::VeloCmd::ConstPtr& msg);
        // void trajVeloCtrlCallback(const geometry_msgs::Twist::ConstPtr& msg);

        /* ----------- Timer callback function ----------- */
        void convertMsg2PipeCallback(const ros::TimerEvent& event);
        void userInputGoalPointsCallback(const ros::TimerEvent& event);
        void pidCtrlCallback(const ros::TimerEvent& event);
        // void sendMsgToVisionCallback(const ros::TimerEvent& event);
        void pidCtrlCallback_yaw(const ros::TimerEvent& event);
        void stopCallback(const ros::TimerEvent& event);

        /* ----------- Frame transform function ----------- */
        void createLocalFrame();
        /* position_cmd command */


        /* /////--------------------- Data transform from sensor ---------------------///// */
        // convert vehicle position from world ENU to local FLU
        Eigen::Vector3d convertVehiclePositionWorldENU2LocalFLU(Eigen::Vector3d posi_enu);
        // convert vehicle attitude from world ENU to local FLU
        Eigen::Quaterniond convertVehicleAttitudeWorldENU2LocalFLU(Eigen::Quaterniond quat_enu);
        // convert vehicle velocity from world ENU to local FLU
        Eigen::Vector3d convertVehicleVelocityWorldENU2LocalFLU(Eigen::Vector3d velo_enu);
        // convert gimbal attitude from world ENU to local FLU
        Eigen::Quaterniond convertGimbalAttitudeWorldENU2LocalFLU(Eigen::Quaterniond quat_enu);

        /* data transform to control */
        // convert vehicle velo from local FLU to world ENU
        Eigen::Vector3d convertVehicleVelocityLocalFLU2WorldENU(Eigen::Vector3d velo_flu);
        // convert vehicle yaw dot from local FLU to world ENU
        double convertVehicleYawDotLocalFLU2WorldENU(double yaw_flu);

        /* update debug message(odom, frame ...) */
        void updateDebugMasterMsg();

    private:

        ros::NodeHandle                             nh_;

        std::atomic_flag                            mutex_ = ATOMIC_FLAG_INIT;

        double                                      rate_task_ctrl_;

        // time
        ros::Time                                   info_time_;

        // velocity command
        // insp_msgs::VeloCmd                          set_velo_;

        // global data
        Eigen::Vector3d                             global_posi_;
        Eigen::Quaterniond                          global_quat_;
        Eigen::Vector3d                             global_euler_;
        Eigen::Vector3d                             global_velo_;

        // local data
        Eigen::Vector3d                             local_posi_;
        Eigen::Quaterniond                          local_quat_;
        Eigen::Vector3d                             local_euler_;
        Eigen::Vector3d                             local_velo_;
        Eigen::Quaterniond                          local_gimbal_quat_;
        Eigen::Vector3d                             local_gimbal_euler_;

        // set data
        Eigen::Vector4d                             set_local_velo_;
        Eigen::Vector3d                             set_local_gimbal_vec_;

        Eigen::Vector4d                             set_global_velo_;
        Eigen::Vector3d                             set_global_gimbal_vec_;
        Eigen::Vector3d                             set_global_gimbal_euler_;

        // R and t

        
        // world ENU to local FLU
        Eigen::Matrix3d                             R_WorldENU2LocalFLU_;
        Eigen::Vector3d                             t_WorldENU2LocalFLU_;
        Eigen::Matrix4d                             Rt_WorldENU2LocalFLU_;
        Eigen::Matrix3d                             R_LocalFLU2WorldENU_;
        Eigen::Vector3d                             t_LocalFLU2WorldENU_;
        Eigen::Matrix4d                             Rt_LocalFLU2WorldENU_;


        int                                         flight_task_;
        ros::Time                                   last_req_time_;
        


        // std::shared_ptr<visualization::Visualization>    visPtr_;

        std::string                                 str_topic_world_x_;
        std::string                                 str_topic_world_y_;
        std::string                                 str_topic_world_z_;
        std::string                                 str_topic_mavros_x_;
        std::string                                 str_topic_mavros_y_;
        std::string                                 str_topic_mavros_z_;

        std::string                                 str_topic_gim_;

        double                                      arrow_width_;


    };
}

#endif