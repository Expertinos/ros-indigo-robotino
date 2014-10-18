cmake_minimum_required(VERSION 2.8.3)
project(robotino_node)

find_package(catkin REQUIRED COMPONENTS 
	image_transport 
	nav_msgs 
	pcl_conversions 
	pcl_ros 
	robotino_description	
	robotino_msgs 
	tf
	)
set(CMAKE_MODULE_PATH /usr/local/robotino/api2/cmake)
find_package(RobotinoAPI2 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${ROBOTINOAPI2_INCLUDE_DIR})

catkin_package(CATKIN_DEPENDS robotino_msgs)

add_executable(robotino_node 
	src/robotino_node.cpp
	src/AnalogInputArrayROS.cpp
	src/BumperROS.cpp
	src/CameraROS.cpp
	src/CompactBHAROS.cpp
	src/ComROS.cpp
	src/DigitalInputArrayROS.cpp
	src/DigitalOutputArrayROS.cpp
  	src/DistanceSensorArrayROS.cpp
	src/ElectricalGripperROS.cpp
	src/EncoderInputROS.cpp
	src/GrapplerROS.cpp
	#src/KinectROS.cpp
	src/LaserRangeFinderROS.cpp
	src/MotorArrayROS.cpp
	src/NorthStarROS.cpp
	src/OdometryROS.cpp
	src/OmniDriveROS.cpp
	src/PowerManagementROS.cpp
	src/RobotinoNode.cpp
	)

target_link_libraries(robotino_node
	${catkin_LIBRARIES}
	${REC_ROBOTINO_API2_LIBRARY}
	)

add_dependencies(robotino_node robotino_msgs_gencpp)

add_executable(robotino_odometry_node
	src/robotino_odometry_node.cpp
	src/ComROS.cpp
	src/OdometryROS.cpp
	src/RobotinoOdometryNode.cpp
	)

target_link_libraries(robotino_odometry_node
	${catkin_LIBRARIES}
	${REC_ROBOTINO_API2_LIBRARY}
	)

add_dependencies(robotino_odometry_node robotino_msgs_gencpp)

add_executable(robotino_laserrangefinder_node
	src/robotino_laserrangefinder_node.cpp
	src/ComROS.cpp
	src/LaserRangeFinderROS.cpp
	src/RobotinoLaserRangeFinderNode.cpp
	)

target_link_libraries(robotino_laserrangefinder_node
	${catkin_LIBRARIES}
	${REC_ROBOTINO_API2_LIBRARY}
	)

add_executable(robotino_camera_node
	src/robotino_camera_node.cpp
	src/ComROS.cpp
	src/CameraROS.cpp
	src/RobotinoCameraNode.cpp
	)

target_link_libraries(robotino_camera_node
	${catkin_LIBRARIES}
	${REC_ROBOTINO_API2_LIBRARY}
	)

#add_executable(robotino_mapping_node
#	src/robotino_mapping_node.cpp
#	src/MappingROS.cpp
#	src/RobotinoMappingNode.cpp
#	src/InitialPoseROS.cpp
#	src/NavGoalROS.cpp
#	src/ComROS.cpp
#	)

#target_link_libraries(robotino_mapping_node
#	${catkin_LIBRARIES}
#	${REC_ROBOTINO_API2_LIBRARY}
#	)
