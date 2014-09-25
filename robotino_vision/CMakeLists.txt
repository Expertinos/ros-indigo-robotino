cmake_minimum_required(VERSION 2.8.3)
project(robotino_vision)

find_package(catkin REQUIRED COMPONENTS std_msgs roscpp robotino_msgs image_transport cv_bridge)
find_package(OpenCV REQUIRED)
set(CMAKE_MODULE_PATH /usr/local/robotino/api2/cmake)
find_package(RobotinoAPI2 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${ROBOTINOAPI2_INCLUDE_DIR})

catkin_package()

add_executable(robotino_vision_node 
	src/robotino_vision_node.cpp 
	src/RobotinoVision.cpp 
	src/ComROS.cpp 
	src/CameraROS.cpp
	)

target_link_libraries(robotino_vision_node
	${REC_ROBOTINO_API2_LIBRARY}
  	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	)
