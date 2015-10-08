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
	pid_vel_x_(0, 1, 0.01, 0, MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY, 8, 0.02),
	pid_vel_y_(0, 1, 0.01, 0, MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY, 8, 0.02),
	pid_vel_phi_(0, 0.2, 0.1, 0, MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY, 8, 0.01)
{
	readParameters();
	state_ = moveStates::UNINITIALIZED;
	percentage_ = 0;
	
	/* para sintonia dos PIDs */
	kp_x_ = 25, ki_x_ = 0, kd_x_ = 0;
	kp_y_ = 25, ki_y_ = 0, kd_y_ = 0;
	kp_phi_ = 10, ki_phi_ = 0, kd_phi_ = 0;
	double kp_max = 3.1;
	int resolution = 100;
	cv::createTrackbar("Kp of x: ", PID_WINDOW, &kp_x_, resolution);
	cv::createTrackbar("Ki of x: ", PID_WINDOW, &ki_x_, resolution);
	cv::createTrackbar("Kd of x: ", PID_WINDOW, &kd_x_, resolution);
	double kp_x = kp_max * kp_x_ / resolution;
	pid_vel_x_.setKp(kp_x);
	pid_vel_x_.setKi(0);//.01 * ki_x_ / resolution);
	pid_vel_x_.setKd(0);//.01 * kd_x_ / resolution);
	cv::createTrackbar("Kp of y: ", PID_WINDOW, &kp_y_, resolution);
	cv::createTrackbar("Ki of y: ", PID_WINDOW, &ki_y_, resolution);
	cv::createTrackbar("Kd of y: ", PID_WINDOW, &kd_y_, resolution);
	double kp_y = kp_max * kp_y_ / resolution;
	pid_vel_y_.setKp(kp_y);
	pid_vel_y_.setKi(0);//.01 * ki_y_ / resolution);
	pid_vel_y_.setKd(0);//.01 * kd_y_ / resolution);
	cv::createTrackbar("Kp of phi: ", PID_WINDOW, &kp_phi_, resolution);
	cv::createTrackbar("Ki of phi: ", PID_WINDOW, &ki_phi_, resolution);
	cv::createTrackbar("Kd of phi: ", PID_WINDOW, &kd_phi_, resolution);
	double kp_phi = kp_max * kp_phi_ / resolution;
	pid_vel_phi_.setKp(kp_phi);
	pid_vel_phi_.setKi(0);//.01 * ki_phi_ / resolution);
	pid_vel_phi_.setKd(0);//.01 * kd_phi_ / resolution);
	cv::waitKey(30);
	cv::namedWindow(PID_WINDOW);
	/**************************/
}

/**
 *
 */
MoveServer::~MoveServer() 
{
	cv::destroyAllWindows();
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
	/* para sintonia dos PIDs */
	double kp_max = 3.1;
	int resolution = 100;
	cv::createTrackbar("Kp of x: ", PID_WINDOW, &kp_x_, resolution);
	cv::createTrackbar("Ki of x: ", PID_WINDOW, &ki_x_, resolution);
	cv::createTrackbar("Kd of x: ", PID_WINDOW, &kd_x_, resolution);
	double kp_x = kp_max * kp_x_ / resolution;
	pid_vel_x_.setKp(kp_x);
	pid_vel_x_.setKi(0);//.01 * ki_x_ / resolution);
	pid_vel_x_.setKd(0);//.01 * kd_x_ / resolution);
	cv::createTrackbar("Kp of y: ", PID_WINDOW, &kp_y_, resolution);
	cv::createTrackbar("Ki of y: ", PID_WINDOW, &ki_y_, resolution);
	cv::createTrackbar("Kd of y: ", PID_WINDOW, &kd_y_, resolution);
	double kp_y = kp_max * kp_y_ / resolution;
	pid_vel_y_.setKp(kp_y);
	pid_vel_y_.setKi(0);//.01 * ki_y_ / resolution);
	pid_vel_y_.setKd(0);//.01 * kd_y_ / resolution);
	cv::createTrackbar("Kp of phi: ", PID_WINDOW, &kp_phi_, resolution);
	cv::createTrackbar("Ki of phi: ", PID_WINDOW, &ki_phi_, resolution);
	cv::createTrackbar("Kd of phi: ", PID_WINDOW, &kd_phi_, resolution);
	double kp_phi = kp_max * kp_phi_ / resolution;
	pid_vel_phi_.setKp(kp_phi);
	pid_vel_phi_.setKi(0);//.01 * ki_phi_ / resolution);
	pid_vel_phi_.setKd(5.1 * kd_phi_ / resolution);
	cv::waitKey(30);
	/*************************/

	double vel_x = 0, vel_y = 0, vel_phi = 0;
	double error_x = 0, error_y = 0, error_phi = 0;
	error_x = pid_vel_x_.getError();
	pid_vel_x_.compute(getOdometry_X());
	vel_x = pid_vel_x_.getInput();
	error_y = pid_vel_y_.getError();
	pid_vel_y_.compute(getOdometry_Y());
	vel_y = pid_vel_y_.getInput();
	error_phi = pid_vel_phi_.getError();
	pid_vel_phi_.compute(getOdometry_PHI());
	vel_phi = pid_vel_phi_.getInput();
	
	ROS_WARN("vx=%f,vy=%f,vphi=%f", vel_x, vel_y, vel_phi);
	ROS_ERROR("ex=%f,ey=%f,ephi=%f", error_x, error_y, 180 * error_phi / PI);
	ROS_INFO("kpx=%f,kpy=%f,kpphi=%f", kp_x, kp_y, kp_phi);

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
	pid_vel_phi_.setSetPoint(0);//goal->delta_phi);
	path_mode_ = PathModes::newInstance(goal->path_mode);
	velocity_mode_ = VelocityModes::newInstance(goal->velocity_mode);
	percentage_ = 0;
	resetOdometry();
	pid_vel_x_.compute(getOdometry_X());
	pid_vel_y_.compute(getOdometry_Y());
	pid_vel_phi_.compute(getOdometry_PHI());
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
