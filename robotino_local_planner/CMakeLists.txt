cmake_minimum_required(VERSION 2.8.3)
project(robotino_local_planner)

find_package(catkin REQUIRED COMPONENTS
	costmap_2d
	geometry_msgs
	nav_msgs
	nav_core
	tf
	pluginlib
	)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(
  RobotinoLocalPlanner
  src/RobotinoLocalPlanner.cpp )

target_link_libraries(RobotinoLocalPlanner
   ${catkin_LIBRARIES}
)
