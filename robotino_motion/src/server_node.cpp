/*
 * robotino_motion_server_node.cpp
 *
 *  Created on: 2014
 *      Author: expertinos.unifei@gmail.com
 */

#include "Server.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "server_node");
	ros::NodeHandle nh;
	Server node(nh, "", "") ;
	node.spin();

	return 0;
}
