cmake_minimum_required(VERSION 2.8.3 FATAL_ERROR)

set (CMAKE_CXX_STANDARD 11)

project(relocalization_evaluation)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs rosbag pcl_ros)
find_package(PCL 1.8 REQUIRED COMPONENTS common io visualization filters)

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(relocalization_evaluation relocalization_evaluation.cpp)

target_link_libraries(relocalization_evaluation ${catkin_LIBRARIES} ${PCL_LIBRARIES})

