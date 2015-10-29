/*
 * read_order_server_node.cpp
 *
 *  Created on: 10/2015
 *  Author: adrianohrl@unifei.edu.br
 *  Maintainer: expertinos.unifei@gmail.com
 */

#include "ReadOrderServer.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "read_order_server_node");
	ros::NodeHandle nh;
	ReadOrderServer node(nh) ;
	node.spin();

	return 0;
}
