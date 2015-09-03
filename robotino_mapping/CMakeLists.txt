cmake_minimum_required(VERSION 2.8.3)
project(robotino_mapping)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	robotino_motion
	robotino_planner
	roscpp
	std_msgs
	)
find_package(OpenCV REQUIRED)

add_service_files(
	FILES
	GetCoordinates.srv
	GetIndexes.srv
	GetMap.srv
	GetAreaCoordinates.srv
	SetMap.srv
	)

generate_messages(
	DEPENDENCIES
	std_msgs
	)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(robotino_mapping_node 
	src/robotino_mapping_node.cpp
	src/RobotinoMapping.cpp
	)

add_dependencies(robotino_mapping_node
	robotino_mapping_gen_cpp
	${robotino_mapping_EXPORTED_TARGETS}
	)

target_link_libraries(robotino_mapping_node
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	)
