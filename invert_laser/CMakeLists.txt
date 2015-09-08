cmake_minimum_required(VERSION 2.8.3)
project(invert_laser)

find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	sensor_msgs
	roscpp
	)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
	
add_executable(
	invert_laser_node 
	src/invert_laser_node.cpp
	src/invert_laser.cpp)

target_link_libraries(invert_laser_node
	${catkin_LIBRARIES}
	)


