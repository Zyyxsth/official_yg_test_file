cmake_minimum_required(VERSION 3.5)
project(diablo_body)
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


find_package(
        catkin REQUIRED
        motion_msgs
        roscpp
        diablo_utils
        sensor_msgs
        std_msgs
        geometry_msgs
    )


include_directories(include)

set(library_name ${PROJECT_NAME}_core)


add_library(${library_name} SHARED
  src/diablo_imu.cpp
  # src/diablo_battery.cpp
  src/diablo_legmotors.cpp
  src/diablo_body_state.cpp
  src/diablo_battery.cpp
)
target_link_libraries(${library_name} 
  ${catkin_LIBRARIES}
)

# target_include_directories(${})
target_include_directories(${library_name} PUBLIC
  ${catkin_INCLUDE_DIRS}
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${library_name}
)

catkin_install_python(PROGRAMS test.py testremote.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS testmpc.py remoteinfo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS rotation.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS single_mpc.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

