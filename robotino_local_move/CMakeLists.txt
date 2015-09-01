cmake_minimum_required(VERSION 2.8.3)
project(robotino_local_move)

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
	roscpp
	)

find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
	FILES
	#LocalMoveAction.msg
	#LocalMoveActionFeedback.msg
	#LocalMoveActionGoal.msg
	#LocalMoveActionResult.msg
	#LocalMoveFeedback.msg
	#LocalMoveGoal.msg
	#LocalMoveResult.msg
	PathDisplacements.msg
	PathStatus.msg
	RobotPos.msg
	)

add_service_files(
	FILES
	FullPath.srv
	Stop.srv
	)

add_action_files(
	DIRECTORY action
	FILES 
	LocalMove.action
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

add_executable( robotino_local_move_node 
 	src/robotino_local_move_node.cpp
  	src/RobotinoLocalMoveServer.cpp
  	src/RobotinoLocalMoveClient.cpp
	)
target_link_libraries( robotino_local_move_node
	${catkin_LIBRARIES}
	)
add_dependencies(robotino_local_move_node
	${robotino_local_move_EXPORTED_TARGETS}
	)



add_executable( robotino_manager 
	src/robotino_manager.cpp
	)
target_link_libraries( robotino_manager
	${catkin_LIBRARIES}
	)
add_dependencies(robotino_manager
	${robotino_local_move_EXPORTED_TARGETS}
	)



add_executable( manager_client 
	src/manager_client.cpp
	)
target_link_libraries( manager_client
	${catkin_LIBRARIES}
	)
add_dependencies(manager_client
	${robotino_local_move_EXPORTED_TARGETS}
	)

