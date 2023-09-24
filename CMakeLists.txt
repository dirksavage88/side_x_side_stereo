cmake_minimum_required(VERSION 3.5)

project(side_x_side_stereo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(rclcpp REQUIRED)
find_package(image_common REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
# find_package(camera_info_manager REQURIED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)

include_directories(include ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${camera_info_manager_INCLUDE_DIRS})
## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/side_by_side_stereo_node.cpp)

ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs OpenCV image_common image_transport cv_bridge camera_info_manager)

# Install include directories
# install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

# ament_export_include_directories(${PROJECT_NAME}_node/include)
ament_export_dependencies(cv_bridge image_transport rclcpp sensor_msgs std_msgs)

ament_package()
