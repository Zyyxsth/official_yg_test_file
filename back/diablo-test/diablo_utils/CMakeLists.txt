cmake_minimum_required(VERSION 3.5)
project(diablo_utils)
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
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_definitions(-D_LOG_CONSOLE)


include_directories(include)

set(library_name ${PROJECT_NAME}_core)

find_package(
        catkin REQUIRED
)

add_library(${library_name} SHARED
  src/SerialPort/SerialPort.cpp
  src/diablo_tools/osdk_crc.cpp
  src/diablo_tools/osdk_hal.cpp
  src/diablo_tools/osdk_log.cpp
  src/diablo_tools/osdk_movement.cpp
  src/diablo_tools/osdk_telemetry.cpp
  src/diablo_tools/osdk_vehicle.cpp
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${library_name}
)

# /opt/ros/noetic/lib/libroscpp.so/usr/lib/aarch64-linux-gnu/libpthread.so/usr/lib/aarch64-linux-gnu/libboost_chrono.so.1.71.0/usr/lib/aarch64-linux-gnu/libboost_filesystem.so.1.71.0/opt/ros/noetic/lib/librosconsole.so/opt/ros/noetic/lib/librosconsole_log4cxx.so/opt/ros/noetic/lib/librosconsole_backend_interface.so/usr/lib/aarch64-linux-gnu/liblog4cxx.so/usr/lib/aarch64-linux-gnu/libboost_regex.so.1.71.0/opt/ros/noetic/lib/libxmlrpcpp.sodiablo_utils_core/opt/ros/noetic/lib/libroscpp_serialization.so/opt/ros/noetic/lib/librostime.so/usr/lib/aarch64-linux-gnu/libboost_date_time.so.1.71.0/opt/ros/noetic/lib/libcpp_common.so/usr/lib/aarch64-linux-gnu/libboost_system.so.1.71.0/usr/lib/aarch64-linux-gnu/libboost_thread.so.1.71.0/usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4

