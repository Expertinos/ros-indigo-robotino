/*
 * RobotinoPlannerClient.h
 *
 *  Created on: 12.10.2014
 *      Author: adrianohrl@unifei.edu.br
 */

#ifndef ROBOTINOPLANNERCLIENT_H_
#define ROBOTINOPLANNERCLIENT_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "robotino_planner/PlannerGoal.h"
#include "robotino_planner/PlannerAction.h"
#include "robotino_planner/Abort.h"
#include "robotino_planner/StartModule.h"


typedef actionlib::SimpleActionClient<robotino_planner::PlannerAction> Client;

class RobotinoPlannerClient
{
public:
	RobotinoPlannerClient();
	~RobotinoPlannerClient();

	bool checkServer();
	void spin();
	void sendGoal(const robotino_planner::PlannerGoal& goal);

private:
	ros::NodeHandle nh_;

	ros::ServiceServer abort_srv_;
	ros::ServiceServer start_srv_;

	Client client_;

	robotino_planner::PlannerGoal goal_;

	bool abort(robotino_planner::Abort::Request &req, robotino_planner::Abort::Response &res);
	bool start(robotino_planner::StartModule::Request &req, robotino_planner::StartModule::Response &res);

};

#endif /* ROBOTINOPLANNERCLIENT_H_ */
