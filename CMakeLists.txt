cmake_minimum_required(VERSION 3.8)
project(gmps_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
include(cmake/dw_version.cmake) # 版の名乗り(dw-notes「版を名乗る」2026-08-22。ビルドのたびに git の <枝>@<hash>[+dirty] を埋め込む)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)
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
 autoware_auto_vehicle_msgs
 can_msgs
 diagnostic_updater
)
dw_version_attach(gmps_driver)

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
