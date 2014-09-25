/*
 * robotino_motion_server_node.cpp
 *
 *  Created on: 14.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#include "RobotinoMotionServer.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "robotino_motion_server_node");
	RobotinoMotionServer rlms;
	rlms.spin();

	return 0;
}
