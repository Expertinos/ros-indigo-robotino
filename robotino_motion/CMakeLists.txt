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

find_package(OpenCV REQUIRED)

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
	FILES Align.action
	      GrabPuck.action	
	      Move.action	
	      StorePuck.action	
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

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(align_server_node 
	src/servers/align_server_node.cpp
	src/Server.cpp
	src/AlignServer.cpp
	src/AlignStates.cpp
	src/AlignAlignmentModes.cpp
	src/AlignDistanceModes.cpp
	)

target_link_libraries(align_server_node
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	)

add_dependencies(align_server_node
	robotino_motion_gen_cpp
	robotino_msgs_gen_cpp
	${robotino_motion_EXPORTED_TARGETS}
	)

add_executable(grab_server_node 
	src/servers/grab_server_node.cpp
	src/Server.cpp
	src/GrabPuckServer.cpp
	src/Colors.cpp
	src/GrabPuckStates.cpp
	)

target_link_libraries(grab_server_node
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	)

add_dependencies(grab_server_node
	robotino_vision_gen_cpp
	robotino_motion_gen_cpp
	robotino_msgs_gen_cpp
	${robotino_motion_EXPORTED_TARGETS}
	)

add_executable(move_server_node 
	src/servers/move_server_node.cpp
	src/Server.cpp
	src/MoveServer.cpp
	src/MoveStates.cpp
	src/MovePathModes.cpp
	src/MoveVelocityModes.cpp
	src/PID.cpp
	)

target_link_libraries(move_server_node
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	)

add_dependencies(move_server_node
	robotino_motion_gen_cpp
	robotino_msgs_gen_cpp
	${robotino_motion_EXPORTED_TARGETS}
	)

############### Action Client Nodes for testing #####################

add_executable(align_client_node 
	src/clients/align_client_node.cpp
	)

target_link_libraries(align_client_node
	${catkin_LIBRARIES}
	)

add_dependencies(align_client_node
	robotino_motion_gen_cpp
	${robotino_motion_EXPORTED_TARGETS}
	)

add_executable(grab_client_node 
	src/clients/grab_client_node.cpp
	)

target_link_libraries(grab_client_node
	${catkin_LIBRARIES}
	)

add_dependencies(grab_client_node
	robotino_motion_gen_cpp
	${robotino_motion_EXPORTED_TARGETS}
	)

add_executable(move_client_node 
	src/clients/move_client_node.cpp
	)

target_link_libraries(move_client_node
	${catkin_LIBRARIES}
	)

add_dependencies(move_client_node
	robotino_motion_gen_cpp
	${robotino_motion_EXPORTED_TARGETS}
	)

add_executable(store_client_node 
	src/clients/store_client_node.cpp
	)

target_link_libraries(store_client_node
	${catkin_LIBRARIES}
	)

add_dependencies(store_client_node
	robotino_motion_gen_cpp
	${robotino_motion_EXPORTED_TARGETS}
	)
