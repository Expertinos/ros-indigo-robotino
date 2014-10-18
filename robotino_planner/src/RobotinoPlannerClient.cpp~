/*
 * RobotinoPlannerClient.cpp
 *
 *  Created on: 12.10.2014
 *      Author: adrianohrl@unifei.edu.br
 */

#include "RobotinoPlannerClient.h"

RobotinoPlannerClient::RobotinoPlannerClient():
	client_("planner", false)
{
	abort_srv_ = nh_.advertiseService("abort", &RobotinoPlannerClient::abort, this);
	start_srv_ = nh_.advertiseService("start_module", &RobotinoPlannerClient::start, this);
}

RobotinoPlannerClient::~RobotinoPlannerClient()
{
	abort_srv_.shutdown();
	start_srv_.shutdown();
}


bool RobotinoPlannerClient::start(robotino_planner::StartModule::Request &req, robotino_planner::StartModule::Response &res)
{
	bool succeed = false;
	robotino_planner::PlannerGoal goal;
	goal.module = req.module;

	if ((goal.module == 0 || goal.module == 1) && checkServer())
	{
		ROS_INFO("Sending goal (Module: %d)", goal.module == 0 );
		sendGoal(goal);
		succeed = true;
	}	
	res.hasStarted = succeed;
	return succeed;
}

bool RobotinoPlannerClient::checkServer()
{
	for( int i = 0; i < 5; ++i)
	{
		ros::spinOnce();
		if(client_.waitForServer(ros::Duration(1.0)))
		{
			ROS_INFO("Connected to the Planner Server!!!");
			return true;
		}
		else
		{
			ROS_INFO("Waiting for Planner Server!!!");
		}
	}
	ROS_ERROR("Planner Server is not running!!!");
	return false;
}

void RobotinoPlannerClient::spin()
{
	ros::Rate loop_rate(5);
	ros::Time start_time = ros::Time::now();

	while(nh_.ok())
	{
		if(client_.waitForResult(ros::Duration(1.0)))
		{
			ROS_INFO("Planner succeeded!!!");
			break;
		}
		else
		{
			ROS_INFO("Planner is being executed!!!");
		}
		/*if((ros::Time::now() - start_time).toSec() > max_time_)
		{
			ROS_INFO("Timeout: Aborting Planner");
			client_.cancelAllGoals();
			break;
		}*/

		ros::spinOnce();
		loop_rate.sleep();
	}
}

void RobotinoPlannerClient::sendGoal(const robotino_planner::PlannerGoal& goal)
{
	client_.sendGoal(goal);
	ROS_INFO("Goal sent!!!");
}

bool RobotinoPlannerClient::abort(robotino_planner::Abort::Request &req, robotino_planner::Abort::Response &res)
{
	//////
	return true;
}
