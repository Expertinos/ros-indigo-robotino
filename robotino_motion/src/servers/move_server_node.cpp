/*
 * move_server_node.cpp
 *
 *  Created on: 10/2015
 *  Author: adrianohrl@unifei.edu.br
 *  Maintainer: expertinos.unifei@gmail.com
 */

#include "MoveServer.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "move_server_node");
	ros::NodeHandle nh;
	MoveServer node(nh) ;
	node.spin();

	return 0;
}
