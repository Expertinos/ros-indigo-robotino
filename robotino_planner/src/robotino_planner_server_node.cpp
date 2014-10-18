/*
 * robotino_planner_server_node.cpp
 *
 *  Created on: 12.10.2014
 *      Author: adrianohrl@unifei.edu.br
 */

#include "RobotinoPlannerServer.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "robotino_planner_server_node");
	RobotinoPlannerServer rps;
	rps.spin();

	return 0;
}
