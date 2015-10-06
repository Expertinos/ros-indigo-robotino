/**
 *  MoveServer.h
 *
 *  Version: 1.1.0.0
 *  Created on: 01/10/2015
 *  Modified on: ??/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef MOVE_SERVER_H_
#define MOVE_SERVER_H_

#include <vector>

#include "Server.h"
#include "MoveStates.h"
#include "MovePathModes.h"
#include "MoveVelocityModes.h"
#include "PID.h"

#include "robotino_motion/MoveAction.h"
	
class MoveServer : public Server
{

public:

	MoveServer(ros::NodeHandle nh);
	~MoveServer();

	bool isActing();	

protected:

	void start();
	void stop();
	void controlLoop();

private:

	void readParameters();

	/** Move Action related Variables and Functions */ 	
	actionlib::SimpleActionServer<robotino_motion::MoveAction> server_;
	robotino_motion::MoveFeedback feedback_;
	robotino_motion::MoveResult result_;

	void executeCallback(const robotino_motion::MoveGoalConstPtr& goal);
	bool validateNewGoal(const robotino_motion::MoveGoalConstPtr& goal);
	void publishFeedback();

	/** Movement related Variables and Functions */
	MoveState state_;
	double percentage_; 

	// Back Movement Variables
	PathMode path_mode_;
	VelocityMode velocity_mode_;
	PID pid_vel_x_, pid_vel_y_, pid_vel_phi_;

};

#endif /* MOVE_SERVER_H_ */
