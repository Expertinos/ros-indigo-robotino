cmake_minimum_required(VERSION 2.8.3)
project(robotino_navigation)

find_package(catkin REQUIRED COMPONENTS
	robotino_node
	roscpp
	)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})


