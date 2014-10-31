/*
 * robotino_planner_client_node.cpp
 *
 *  Created on: 12.10.2014
 *      Author: adrianohrl@unifei.edu.br
 */

#include "RobotinoPlannerClient.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "robotino_planner_client_node");
	ros::NodeHandle nh_;

	RobotinoPlannerClient rpc;
	robotino_planner::PlannerGoal goal;

	if (argc == 2)
	{
		std::istringstream module(argv[1]);
		module >> goal.module;
		rpc.sendGoal(goal);
	}
	ros::spin();

	return 0;
}
