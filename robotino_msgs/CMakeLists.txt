cmake_minimum_required(VERSION 2.8.3)
project(robotino_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp message_generation)

add_message_files(
	FILES
	AnalogReadings.msg
	BHAReadings.msg
	DigitalReadings.msg
	EncoderReadings.msg
	GrapplerReadings.msg
	GripperState.msg
	MotorReadings.msg
	NorthStarReadings.msg
	PowerReadings.msg
	SetBHAPressures.msg
	SetGrapplerAxes.msg
	SetGrapplerAxis.msg
	)

add_service_files(
	FILES
	ResetOdometry.srv
	SetEncoderPosition.srv
	SetGripperState.srv
	)

generate_messages(
	DEPENDENCIES
	geometry_msgs
	)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})
