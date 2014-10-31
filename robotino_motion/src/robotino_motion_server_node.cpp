/*
 * robotino_motion_server_node.cpp
 *
 *  Created on: 2014
 *      Author: expertinos.unifei@gmail.com
 */

#include "RobotinoMotionServer.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "robotino_motion_server_node");
	RobotinoMotionServer rms;
	rms.spin();

	return 0;
}
