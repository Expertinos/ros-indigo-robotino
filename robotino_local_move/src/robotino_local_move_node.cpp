/*
 * robotino_local_move_server_node.cpp
 *
 *  Created on: 17/07/2014
 *      Author: adrianohrl@unifei.edu.br
 */

#include "RobotinoLocalMoveServer.h"
#include "RobotinoLocalMoveClient.h"
#include "robotino_local_move/PathDisplacements.h"
#include "robotino_local_move/Stop.h"
#include "robotino_local_move/PathStatus.h"

#define PI 3.14159
#define MAX_TIME 60

ros::Publisher status_pub;

bool stopCallback(robotino_local_move::Stop::Request &req, robotino_local_move::Stop::Response &res);

void pathCallback(const robotino_local_move::PathDisplacements::ConstPtr& msg);

void sendStatus(int status);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "robotino_local_move_node");
	ros::NodeHandle n;

	ros::Subscriber path_sub = n.subscribe("path", 1000, pathCallback);
	ros::ServiceServer stop_srv = n.advertiseService("stop", stopCallback);
	status_pub = n.advertise<robotino_local_move::PathStatus>("path/status", 1);

	RobotinoLocalMoveServer rlms;
	rlms.spin();
	return 0;
}

bool stopCallback(robotino_local_move::Stop::Request &req,
         robotino_local_move::Stop::Response &res)
{
	RobotinoLocalMoveClient rlmc;
	rlmc.cancelGoal();
	res.result = true;
	return true;
}

void pathCallback(const robotino_local_move::PathDisplacements::ConstPtr& msg)
{	
	RobotinoLocalMoveClient rlmc;
	robotino_local_move::LocalMoveGoal goal;
	goal.move_x = msg->move_x;
	goal.move_y = msg->move_y;
	goal.rotation = (msg->rotation * PI) / 180; // To radians
	goal.movementType = msg->movementType;
	rlmc.setMaxTime(MAX_TIME);

	ROS_INFO("Sending goal (move_x[m], move_y[m], rotate[deg], movementType)=(%f, %f, %f, %d)",
			goal.move_x, goal.move_y, goal.rotation, goal.movementType);

	if (rlmc.checkServer())
	{
		rlmc.sendGoal(goal);
		int status = rlmc.spin();
		sendStatus(status);
	}
}

void sendStatus(int status)
{
	robotino_local_move::PathStatus status_msg;
	status_msg.status = status;
	status_pub.publish(status_msg);
}
