/**
 *  MoveServer.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 01/10/2015
 *  Modified on: ??/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "MoveServer.h"

/**
 *
 */
MoveServer::MoveServer(ros::NodeHandle nh) : 
	Server(nh, "Move"),
	server_(nh, "move", boost::bind(&MoveServer::executeCallback, this, _1), false),
	pid_vel_x_(0, 0.9, 0, 0, MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY, 20, 0.02),
	pid_vel_y_(0, 0.9, 0, 0, MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY, 20, 0.02),
	pid_vel_phi_(0, 10, 0, 0, MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY, 20, 0.01)
{
	readParameters();
	state_ = moveStates::UNINITIALIZED;
	percentage_ = 0;
}

/**
 *
 */
MoveServer::~MoveServer() 
{
	server_.shutdown();
}

/**
 *
 */
bool MoveServer::isActing()
{
	return state_ != moveStates::UNINITIALIZED && state_ != moveStates::IDLE;
}

/**
 *
 */
void MoveServer::start()
{
	if (state_ == moveStates::UNINITIALIZED)
	{
		server_.start();
		state_ = moveStates::IDLE;
	}
}

/**
 *
 */
void MoveServer::stop()
{
	setVelocity(0, 0, 0);
	publishVelocity();
	state_ = moveStates::IDLE;
	result_.goal_achieved = false;
	result_.message = "Unexpected emergency stop request!!!";
	server_.setAborted(result_, result_.message);
}

/**
 *
 */
void MoveServer::controlLoop()
{	
	//double vel_x = 0, vel_y = 0, vel_phi = 0;
	pid_vel_x_.compute(getOdometry_X());
	double vel_x = pid_vel_x_.getInput();
	double error_x = pid_vel_x_.getError();
	pid_vel_y_.compute(getOdometry_Y());
	double vel_y = pid_vel_y_.getInput();
	double error_y = pid_vel_y_.getError();
	pid_vel_phi_.compute(getOdometry_PHI());
	double vel_phi = pid_vel_phi_.getInput();
	double error_phi = pid_vel_phi_.getError();
	if (pid_vel_x_.isInSteadyState() && pid_vel_y_.isInSteadyState() && pid_vel_phi_.isInSteadyState())	
	{
		percentage_ = 100;
		state_ = moveStates::FINISHED;
	}
	setVelocity(vel_x, vel_y, vel_phi);
	publishVelocity();
	publishFeedback();
}

/**
 *
 */
void MoveServer::executeCallback(const robotino_motion::MoveGoalConstPtr& goal)
{
	ros::Rate loop_rate(20);
	if(!validateNewGoal(goal))
	{
		ROS_WARN("Goal not accepted!!!");
		return;
	}
	while(nh_.ok())
	{
		if(server_.isPreemptRequested())
		{
			if(server_.isNewGoalAvailable() && !validateNewGoal(server_.acceptNewGoal()))
			{
				ROS_WARN("Goal not accepted!!!");
				return;
			}
			ROS_INFO("Cancel request!!!");
			server_.setPreempted();
			state_ = moveStates::IDLE;
			setVelocity(0, 0, 0);
			publishVelocity();
			return;
		}
		controlLoop();
		if(state_ == moveStates::FINISHED)
		{
			state_ = moveStates::IDLE;
			setVelocity(0, 0, 0);
			publishVelocity();
			result_.goal_achieved = true;
			result_.message = "Goal achieved with success!!!";
			server_.setSucceeded(result_);
			return;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	state_ = moveStates::IDLE;
	setVelocity(0, 0, 0);
	publishVelocity();
	result_.goal_achieved = false;
	result_.message = "Aborting on the goal because the node has been killed!!!";
	server_.setAborted(result_, result_.message);
}

/**
 *
 */
bool MoveServer::validateNewGoal(const robotino_motion::MoveGoalConstPtr& goal)
{
	if(state_ == moveStates::UNINITIALIZED)
	{
		result_.goal_achieved = false;
		result_.message = "Odometry not initialized yet!!!";
		server_.setAborted(result_, result_.message);
		ROS_ERROR("%s", result_.message.c_str());
		return false;
	}
	pid_vel_x_.setSetPoint(goal->delta_x);
	pid_vel_y_.setSetPoint(goal->delta_y);
	pid_vel_phi_.setSetPoint(goal->delta_phi);
	path_mode_ = PathModes::newInstance(goal->path_mode);
	velocity_mode_ = VelocityModes::newInstance(goal->velocity_mode);
	percentage_ = 0;
	resetOdometry();
	state_ = moveStates::MOVING;
	ROS_INFO("Goal accepted, moving in %s %s mode to x: %f, y: %f, phi: %f!!!", PathModes::toString(path_mode_).c_str(), VelocityModes::toString(velocity_mode_).c_str(), goal->delta_x, goal->delta_y, 180 * goal->delta_phi / PI);
	return true;
}

/**
 *
 */
void MoveServer::publishFeedback()
{
	feedback_.percentage = percentage_;
	feedback_.state = MoveStates::toString(state_);
	server_.publishFeedback(feedback_);
}

/**
 *
 */
void MoveServer::readParameters() 
{

}
