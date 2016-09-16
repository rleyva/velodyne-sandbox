cmake_minimum_required(VERSION 2.8.3)
project(velodyne_sb)

#add_definitions(-std=c++11)

set(Boost_ADDITIONAL_VERSIONS "1.54" "1.54.0")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

#find_package(Boost REQUIRED COMPONENTS 
#		   system iostreams filesystem thread date_time iostreams serialization timer)

find_package(PCL REQUIRED)

message(STATUS " Library  - ${Boost_LIBRARY_DIRS}")

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib
)

# Point-cloud Filtering 
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(velodyne_sb_node src/velodyne_filtering.cpp)

target_link_libraries(velodyne_sb_node
   ${PCL_LIBRARIES}
   ${Boost_LIBRARIES}
   ${catkin_LIBRARIES}
)

# RANSAC segmentation
add_executable(ransac_sphere_node src/ransac_buoy.cpp)

target_link_libraries(ransac_sphere_node
   ${PCL_LIBRARIES}
   ${Boost_LIBRARIES}
   ${catkin_LIBRARIES}
)

