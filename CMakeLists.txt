cmake_minimum_required(VERSION 3.5)
project(slam_karto)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sparse_bundle_adjustment REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(include ${EIGEN3_INCLUDE_DIRS})
add_definitions(${EIGEN3_DEFINITIONS})

add_executable(slam_karto 
src/slam_karto.cpp 
src/spa_solver.cpp 
src/Mapper.cpp 
src/Karto.cpp 
)

ament_target_dependencies(slam_karto 
rclcpp
sparse_bundle_adjustment   
nav_msgs
sensor_msgs
visualization_msgs
tf2
tf2_ros
tf2_msgs
tf2_geometry_msgs
Eigen3
Boost)

install(TARGETS slam_karto
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
