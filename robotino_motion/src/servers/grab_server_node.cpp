/*
 * grab_server_node.cpp
 *
 *  Created on: 10/2015
 *  Author: adrianohrl@unifei.edu.br
 *  Maintainer: expertinos.unifei@gmail.com
 */

#include "GrabPuckServer.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "grab_server_node");
	ros::NodeHandle nh;
	GrabPuckServer node(nh, "") ;
	node.spin();

	return 0;
}
