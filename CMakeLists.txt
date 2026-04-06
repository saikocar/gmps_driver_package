cmake_minimum_required(VERSION 3.8)
project(gmps_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(gmps_msgs REQUIRED) 
find_package(can_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

add_executable(gmps_driver
 src/gmps_driver.cpp
)
ament_target_dependencies(gmps_driver
 rclcpp
 gmps_msgs
 std_msgs
 geometry_msgs
 autoware_vehicle_msgs
 can_msgs
 diagnostic_updater
)

install(TARGETS
  gmps_driver
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
