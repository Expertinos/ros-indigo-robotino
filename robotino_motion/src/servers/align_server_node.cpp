/*
 * align_server_node.cpp
 *
 *  Created on: 10/2015
 *  Author: adrianohrl@unifei.edu.br
 *  Maintainer: expertinos.unifei@gmail.com
 */

#include "AlignServer.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "align_server_node");
	ros::NodeHandle nh;
	AlignServer node(nh) ;
	node.spin();

	return 0;
}
