cmake_minimum_required(VERSION 3.8)
project(nav3d_px4_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(vio_odometry_bridge src/vio_odometry_bridge.cpp)
target_include_directories(vio_odometry_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(vio_odometry_bridge Eigen3::Eigen)

ament_target_dependencies(vio_odometry_bridge
  rclcpp
  nav_msgs
  px4_msgs
  tf2_ros
  geometry_msgs
)

install(TARGETS vio_odometry_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
