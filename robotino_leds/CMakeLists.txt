cmake_minimum_required(VERSION 2.8.3)
project(robotino_leds)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	message_generation
	)

add_service_files(
	FILES
	Sinalize.srv
	Trigger.srv
	)

generate_messages(
	DEPENDENCIES
	)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(robotino_leds_node 
	src/robotino_leds_node.cpp
	src/RobotinoLeds.cpp
	src/Colors.cpp
	src/SinalizationModes.cpp
	)

target_link_libraries(robotino_leds_node
	${catkin_LIBRARIES}
	)

add_dependencies(robotino_leds_node 
	robotino_msgs_gen_cpp
	robotino_leds_gen_cpp
	${robotino_leds_EXPORTED_TARGETS}
	)
