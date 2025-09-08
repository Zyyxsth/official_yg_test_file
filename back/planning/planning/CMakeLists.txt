cmake_minimum_required(VERSION 3.0.2)
project(planning)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  quadrotor_msgs
  traj_opt
  mapping
  roscpp
  nodelet
  decomp_ros_utils
)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
  CATKIN_DEPENDS quadrotor_msgs traj_opt mapping roscpp nodelet decomp_ros_utils
  LIBRARIES ${PROJECT_NAME}
  DEPENDS EIGEN3
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${DECOMP_UTIL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# planning node
add_library(${PROJECT_NAME}_nodelet
src/${PROJECT_NAME}_nodelet.cpp
src/SIYIClient.cpp # 添加辅助文件的源文件
src/odom_interpolation.cpp
)

target_link_libraries(${PROJECT_NAME}_nodelet
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_nodelet
  ${catkin_EXPORTED_TARGETS}
)

add_executable(traj_server src/traj_server.cpp)
target_link_libraries(traj_server ${catkin_LIBRARIES})
add_dependencies(traj_server ${catkin_EXPORTED_TARGETS})

add_executable(publish_cmd src/publish_cmd.cpp)
target_link_libraries(publish_cmd ${catkin_LIBRARIES})
add_dependencies(publish_cmd ${catkin_EXPORTED_TARGETS})

# add_executable(control_ugv src/control_ugv.cpp)
# target_link_libraries(control_ugv ${catkin_LIBRARIES})
# add_dependencies(control_ugv ${catkin_EXPORTED_TARGETS})

add_executable(play_bag_node src/play_bag_node.cpp)
target_link_libraries(play_bag_node ${catkin_LIBRARIES})
add_dependencies(play_bag_node ${catkin_EXPORTED_TARGETS})

add_executable(odom_interpolation_node src/odom_interpolation.cpp)
target_link_libraries(odom_interpolation_node ${catkin_LIBRARIES})
add_dependencies(odom_interpolation_node ${catkin_EXPORTED_TARGETS})


add_executable(simple_traj_publisher src/simple_traj_publisher.cpp)
target_link_libraries(simple_traj_publisher ${catkin_LIBRARIES})
add_dependencies(simple_traj_publisher ${catkin_EXPORTED_TARGETS})


# add_executable(test_node src/test_node.cpp)
# target_link_libraries(test_node
#   ${catkin_LIBRARIES}
# )