cmake_minimum_required(VERSION 3.5)
project(diablo_ctrl)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-psabi)
endif()
set(executable_name diablo_ctrl_node)
find_package(
        catkin REQUIRED COMPONENTS
        motion_msgs
        roscpp
        diablo_utils
        sensor_msgs
        std_msgs
        geometry_msgs
        diablo_body
    )
catkin_package()
include_directories(include)


add_executable(${executable_name} 
      src/diablo_ctrl.cpp
  )
find_library (libvar diablo_utils_core /home/coax/dia-test/devel/lib/)

target_link_libraries(${executable_name} 
  ${catkin_LIBRARIES}
  # libvar
)
# message(
#   FATAL_ERROR ${catkin_LIBRARIES})

# target_include_directories(${})
target_include_directories(${executable_name} PUBLIC
  ${catkin_INCLUDE_DIRS}
)


# /opt/ros/noetic/lib/libroscpp.so/usr/lib/aarch64-linux-gnu/libpthread.so/usr/lib/aarch64-linux-gnu/libboost_chrono.so.1.71.0/usr/lib/aarch64-linux-gnu/libboost_filesystem.so.1.71.0/opt/ros/noetic/lib/librosconsole.so/opt/ros/noetic/lib/librosconsole_log4cxx.so/opt/ros/noetic/lib/librosconsole_backend_interface.so/usr/lib/aarch64-linux-gnu/liblog4cxx.so/usr/lib/aarch64-linux-gnu/libboost_regex.so.1.71.0/opt/ros/noetic/lib/libxmlrpcpp.so/opt/ros/noetic/lib/libroscpp_serialization.so/opt/ros/noetic/lib/librostime.so/usr/lib/aarch64-linux-gnu/libboost_date_time.so.1.71.0/opt/ros/noetic/lib/libcpp_common.so/usr/lib/aarch64-linux-gnu/libboost_system.so.1.71.0/usr/lib/aarch64-linux-gnu/libboost_thread.so.1.71.0/usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
# /opt/ros/noetic/lib/libroscpp.so/usr/lib/aarch64-linux-gnu/libpthread.so/usr/lib/aarch64-linux-gnu/libboost_chrono.so.1.71.0/usr/lib/aarch64-linux-gnu/libboost_filesystem.so.1.71.0/opt/ros/noetic/lib/librosconsole.so/opt/ros/noetic/lib/librosconsole_log4cxx.so/opt/ros/noetic/lib/librosconsole_backend_interface.so/usr/lib/aarch64-linux-gnu/liblog4cxx.so/usr/lib/aarch64-linux-gnu/libboost_regex.so.1.71.0/opt/ros/noetic/lib/libxmlrpcpp.so/opt/ros/noetic/lib/libroscpp_serialization.so/opt/ros/noetic/lib/librostime.so/usr/lib/aarch64-linux-gnu/libboost_date_time.so.1.71.0/opt/ros/noetic/lib/libcpp_common.so/usr/lib/aarch64-linux-gnu/libboost_system.so.1.71.0/usr/lib/aarch64-linux-gnu/libboost_thread.so.1.71.0/usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
