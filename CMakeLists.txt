cmake_minimum_required(VERSION 3.8)
project(parking_based_on_soc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)

include_directories(include)

add_executable(parking_based_on_soc
  src/parking_based_on_soc.cpp
)

ament_target_dependencies(parking_based_on_soc
  rclcpp
  std_msgs
  geometry_msgs
  autoware_adapi_v1_msgs
)

install(TARGETS parking_based_on_soc
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
