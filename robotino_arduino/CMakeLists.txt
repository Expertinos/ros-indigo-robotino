cmake_minimum_required(VERSION 2.8.3)
project(robotino_arduino)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	roscpp
	rosserial_python
	std_msgs
	)

add_message_files(
	FILES
	Mag.msg
	Ultrasound.msg
	)

add_service_files(
	FILES
	Calibrate.srv
	GetMagDirection.srv
	)

generate_messages(
	DEPENDENCIES 
	std_msgs
	)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(robotino_arduino_node src/robotino_arduino_node.cpp)

target_link_libraries(robotino_arduino_node ${catkin_LIBRARIES})
