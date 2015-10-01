/*
 * robotino_motion_server_node.cpp
 *
 *  Created on: 2014
 *      Author: expertinos.unifei@gmail.com
 */

#include "GrabPuckServer.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "server_node");
	ros::NodeHandle nh;
	GrabPuckServer node(nh, "") ;
	node.spin();

	return 0;
}
