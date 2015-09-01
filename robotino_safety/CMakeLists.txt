cmake_minimum_required(VERSION 2.8.3)
project(robotino_safety)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  laser_geometry
  robotino_node
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
  )

catkin_package( 
	CATKIN_DEPENDS 
	geometry_msgs 
	laser_geometry 
	robotino_node 
	sensor_msgs 
	std_msgs 
	tf 
	visualization_msgs
	)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable( robotino_safety_node
  	src/RobotinoSafety.cpp
  	src/robotino_safety_node.cpp
	)
target_link_libraries( robotino_safety_node
	${catkin_LIBRARIES}
	)
add_dependencies(robotino_safety_node
	robotino_msgs_generate_messages_cpp
	)


