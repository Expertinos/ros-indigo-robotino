cmake_minimum_required(VERSION 2.8.3)
project(robotino_teleop)

find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	robotino_node
	sensor_msgs
	roscpp
	)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

#add_executable(
#	joystick_teleop_node 
#	src/joystick_teleop_node.cpp
#	src/JoystickTeleop.cpp)
	
add_executable(
	keyboard_teleop_node 
	src/keyboard_teleop_node.cpp
	src/KeyboardTeleop.cpp)

target_link_libraries(keyboard_teleop_node
	${catkin_LIBRARIES}
	)


