cmake_minimum_required(VERSION 3.5)
project(motion_msgs)
set(EXPORT_COMPILE_COMMANDS ON)

find_package(
        catkin REQUIRED
        message_generation
        std_msgs
        geometry_msgs 
        #roslib
    )
add_message_files(
  FILES
  CtrlPlot.msg
  LegMotors.msg
  MotionCtrl.msg
  RobotStatus.msg
  MovementCtrlData.msg
  MovementCtrlMode.msg
  Diablo_Ctrl.msg
  Battery.msg
)

# ament_export_dependencies(rosidl_default_runtime std_msgs)

# ament_package()

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs 
)

catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES 
  CATKIN_DEPENDS 
  message_runtime
# DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
