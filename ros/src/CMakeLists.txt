cmake_minimum_required(VERSION 3.16...3.24)
cmake_policy(SET CMP0074 NEW)

project(trg_planner_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

message(STATUS "Configuring trg_planner_ros package")

set(TRG_PLANNER_CORE_PREFIX "${CMAKE_CURRENT_LIST_DIR}/../../install")
if(EXISTS "${TRG_PLANNER_CORE_PREFIX}/lib/cmake/trg_planner/trg_plannerConfig.cmake")
    list(APPEND CMAKE_PREFIX_PATH "${TRG_PLANNER_CORE_PREFIX}")
    message(STATUS "Added trg_planner core prefix: ${TRG_PLANNER_CORE_PREFIX}")
else()
    message(WARNING "Could not find trg_plannerConfig.cmake under ${TRG_PLANNER_CORE_PREFIX}; please verify you have run 'cmake --install' in core/")
endif()

add_definitions(-DTRG_ROS_DIR="${CMAKE_CURRENT_LIST_DIR}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(trg_planner REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${PCL_INCLUDE_DIRS}
)

add_executable(trg_ros2_node
    ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/ros2_node.cpp
)
ament_target_dependencies(trg_ros2_node
    rclcpp
    pcl_conversions
    pcl_ros
    nav_msgs
    visualization_msgs
    trg_planner
)

target_link_libraries(trg_ros2_node
    trg_planner::trg_planner_core
    ${PCL_LIBRARIES}
)

install(TARGETS
    trg_ros2_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    launch
    config
    rviz
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
