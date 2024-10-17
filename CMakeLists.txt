cmake_minimum_required(VERSION 3.8)
project(interview_homework)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
 

add_executable(pb_frames src/publish_frames.cpp)
ament_target_dependencies(
    pb_frames
    geometry_msgs
    rclcpp
    tf2
    tf2_ros
)



install(TARGETS
    pb_frames
    DESTINATION lib/${PROJECT_NAME})
    
install(
  DIRECTORY config launch 
  DESTINATION share/${PROJECT_NAME}
)
    
ament_package()
