cmake_minimum_required(VERSION 2.8.3)
project(robotino_motion)

find_package(catkin REQUIRED COMPONENTS
	actionlib
	actionlib_msgs
	genmsg
	geometry_msgs
	message_generation
	nav_msgs
	robotino_node
	std_msgs
	tf
	)

find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
	FILES
	RobotState.msg
	HasArrived.msg
	)

add_service_files(
	FILES
	Align.srv
	GetProduct.srv
	AchievedGoal.srv
	MoveTo.srv
	SetAchievedGoal.srv
	)

add_action_files(
	DIRECTORY action
	FILES Motion.action
	)

generate_messages(
	DEPENDENCIES
	actionlib_msgs
	std_msgs
	)

catkin_package(
	CATKIN_DEPENDS 
	message_runtime 
	actionlib_msgs
	)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(robotino_motion_server_node 
	src/robotino_motion_server_node.cpp
	src/RobotinoMotionServer.cpp
	)

target_link_libraries(robotino_motion_server_node
	${catkin_LIBRARIES}
	)

add_dependencies(robotino_motion_server_node
	robotino_vision_gen_cpp
	${robotino_motion_EXPORTED_TARGETS}
	)

add_executable(robotino_motion_client_node 
	src/robotino_motion_client_node.cpp
	src/RobotinoMotionClient.cpp
	src/astar.cpp 
	)

target_link_libraries(robotino_motion_client_node
	${catkin_LIBRARIES}
	)

add_dependencies(robotino_motion_client_node
	${robotino_motion_EXPORTED_TARGETS}
	)
