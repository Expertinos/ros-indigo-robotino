cmake_minimum_required(VERSION 2.8.3)
project(robotino_vision)

find_package(catkin REQUIRED COMPONENTS 
	std_msgs 
	roscpp 
	robotino_msgs 
	image_transport 
	cv_bridge 
	message_generation
	)

find_package(OpenCV REQUIRED)

set(CMAKE_MODULE_PATH /usr/local/robotino/api2/cmake)

#set(FL_PATH ../fuzzylite-5.0/fuzzylite)

find_package(RobotinoAPI2 REQUIRED)

add_message_files(
	FILES
	LampPost.msg
)

add_service_files(
  	FILES
	FindObjects.srv
	FindInsulatingTapeAreas.srv
	GetObjectsList.srv
	GetLampPosts.srv
	ContainInList.srv
  	SaveImage.srv
	SetCalibration.srv
	)

generate_messages(
  	DEPENDENCIES
  	std_msgs
	)

include_directories(
	include 
	${catkin_INCLUDE_DIRS} 
	${OpenCV_INCLUDE_DIRS} 
	${ROBOTINOAPI2_INCLUDE_DIR}
#	${FL_PATH}/release/bin
#	${FL_PATH}
	)

link_directories(
#	${FL_PATH}/release/bin 
#	${FL_PATH}
	)

catkin_package(CATKIN_DEPENDS message_runtime)

add_executable(robotino_vision_node 
	src/robotino_vision_node.cpp 
	src/RobotinoVision.cpp 
	src/Colors.cpp
	)

add_dependencies(robotino_vision_node
	robotino_vision_gen_cpp
	${robotino_vision_EXPORTED_TARGETS}
	)

target_link_libraries(robotino_vision_node
	${REC_ROBOTINO_API2_LIBRARY}
  	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	#fuzzylite
	)
