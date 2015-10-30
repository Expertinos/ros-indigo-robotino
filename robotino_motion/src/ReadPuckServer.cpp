/**
 *  ReadPuckServer.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 01/10/2015
 *  Modified on: ??/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ReadPuckServer.h"

/**
 *
 */
ReadPuckServer::ReadPuckServer(ros::NodeHandle nh) : 
	Server(nh, "Read Puck"),
	server_(nh, "read_puck", boost::bind(&ReadPuckServer::executeCallback, this, _1), false)
{
	readParameters();
	find_objects_cli_ = nh_.serviceClient<robotino_vision::FindObjects>("find_objects");

	state_ = read_puck_states::UNINITIALIZED;
	percentage_ = 0;

	verify_markers_ = false;
	final_color_index_ = -1;
}

/**
 *
 */
ReadPuckServer::~ReadPuckServer() 
{
	find_objects_cli_.shutdown();
	server_.shutdown();
}

/**
 *
 */
bool ReadPuckServer::isActing()
{
	return state_ != read_puck_states::UNINITIALIZED && state_ != read_puck_states::IDLE;
}

/**
 *
 */
void ReadPuckServer::start()
{
	if (!find_objects_cli_.exists())
	{
		ROS_WARN("Waiting for robotino_vision_node!!!");
		find_objects_cli_.waitForExistence();
	}
	ROS_INFO("/find_objects service server is running!!!");
	if (state_ == read_puck_states::UNINITIALIZED)
	{
		server_.start();
		state_ = read_puck_states::IDLE;
	}
}

/**
 *
 */
void ReadPuckServer::stop()
{
	setVelocity(0, 0, 0);
	publishVelocity();
	state_ = read_puck_states::IDLE;
	result_.goal_achieved = false;
	result_.message = "Unexpected emergency stop request!!!";
	server_.setAborted(result_, result_.message);
}

/**
 *
 */
void ReadPuckServer::controlLoop()
{
	double percentage = 0;
	/*double vel_x = 0, vel_y = 0, vel_phi = 0;
	ROS_DEBUG("%s", ReadPuckStates::toString(state_).c_str());
	
	if (!loaded_)
	{
		robotino_vision::FindObjects srv;
		srv.request.color = Colors::toCode(color_);
		if (!find_objects_cli_.call(srv))
		{	
			ROS_ERROR("Puck not found!!!");
			state_ = read_puck_states::LOST;
			return;
		}
		std::vector<float> distances = srv.response.distances;
		std::vector<float> directions = srv.response.directions; 
		int num_objects = srv.response.distances.size();
		if (num_objects > 0 && state_ != read_puck_states::GRABBING_PUCK)
		{
			int closest_index = 0;
			for (int i = 0; i < num_objects; i++)
			{
				if (distances[i] < distances[closest_index])
				{
					closest_index = i;
				}
			}
			double max_error_lateral = 50, max_error_frontal = 40;
			double error_lateral = directions[closest_index];
			if (error_lateral > max_error_lateral)
			{
				 error_lateral = max_error_lateral;
			}
			double error_frontal = distances[closest_index];
			if (error_frontal > max_error_frontal)
			{
				 error_frontal = max_error_frontal;
			}
			float tolerance_lateral = 0.1, tolerance_frontal = 35;
			double K_error_lateral = 0.3, K_error_frontal = 0.005;
			double percentage_f, percentage_0, tolerance, max_error, error;
			if (fabs(error_lateral) > tolerance_lateral) // 0% a 49%
			{
				state_ = read_puck_states::ALIGNING_LATERAL;
				vel_y = -K_error_lateral * error_lateral;
				percentage_0 = 0;
				percentage_f = 49;
				tolerance = tolerance_lateral;
				max_error = max_error_lateral;
				error = fabs(error_lateral);
			}
			else if (fabs(error_frontal) > tolerance_frontal) // 50% a 69%
			{
				state_ = read_puck_states::HEADING_TOWARD_PUCK;
				vel_x = K_error_frontal * error_frontal;
				percentage_0 = 50;
				percentage_f = 69;
				tolerance = tolerance_frontal;
				max_error = max_error_frontal;
				error = fabs(error_frontal);
			}
			else 
			{
				vel_x = .14;
				state_ = read_puck_states::GRABBING_PUCK;
				readbing_start_ = ros::Time::now();
			}
			percentage = percentage_0 + (percentage_f - percentage_0) * (max_error - error) / (max_error - tolerance);
		}
		else if (state_ == read_puck_states::GRABBING_PUCK) // 70% a 79%
		{
			state_ = read_puck_states::GRABBING_PUCK;
			vel_x = .14;
			double percentage_0 = 70, percentage_f = 79;
			double elapsed_time = (ros::Time::now() - readbing_start_).toSec();
			if (elapsed_time > GRABBING_DEADLINE)
			{
				state_ = read_puck_states::LOST;
				ROS_ERROR("Readbing state deadline expired!!!");
			}
			percentage = percentage_0 + (percentage_f - percentage_0) * elapsed_time / GRABBING_DEADLINE;
		}		
	}
	else 
	{		
		if (state_ == read_puck_states::GRABBING_PUCK)
		{
			delta_x_ = -getOdometry_X();
			resetOdometry();
			state_ = read_puck_states::ROTATING;
			percentage_ = 79;
		}
		double max_error_linear = 1.25;
		double error_linear = getOdometry_X() - delta_x_;
		if (error_linear > max_error_linear)
		{
			 error_linear = max_error_linear;
		}
		double max_error_angular = PI;
		double error_angular = PI - getOdometry_PHI();
		if (error_angular > max_error_angular)
		{
			 error_angular = max_error_angular;
		}
		float tolerance_linear = 0.1, tolerance_angular = 0.1;
		double K_error_linear = 1.2, K_error_angular = 0.8;
		double percentage_f, percentage_0, tolerance, max_error, error;
		if (fabs(error_angular) > tolerance_angular) // 80% a 89%
		{
			state_ = read_puck_states::ROTATING;
			vel_phi = K_error_angular * error_angular;
			percentage_0 = 80;
			percentage_f = 89;
			tolerance = tolerance_angular;
			max_error = max_error_angular;
			error = fabs(error_angular);
			
		}
		else if (fabs(error_linear) > tolerance_linear) // 90% a 99%
		{
			state_ = read_puck_states::GOING_BACK_TO_ORIGIN;
			vel_x = K_error_linear * error_linear;
			percentage_0 = 91;
			percentage_f = 99;
			tolerance = tolerance_linear;
			max_error = max_error_linear;
			error = fabs(error_linear);			
		}
		percentage = percentage_0 + (percentage_f - percentage_0) * (max_error - error) / (max_error - tolerance);
	}
	if (vel_x == 0 && vel_y == 0 && vel_phi == 0) // 100%
	{
		state_ = read_puck_states::FINISHED;
		percentage_ = 100;
	}
	if (percentage > percentage_)
	{
		percentage_ = percentage;
	}
	setVelocity(vel_x, vel_y, vel_phi);
	publishVelocity();*/
	
	if ((ros::Time::now() - reading_start_).toSec() > READING_DEADLINE) // 100%
	{
		state_ = read_puck_states::FINISHED;
		percentage_ = 100;
	}
	if (percentage > percentage_)
	{
		percentage_ = percentage;
	}
	publishFeedback();
}

/**
 *
 */
void ReadPuckServer::executeCallback(const robotino_motion::ReadPuckGoalConstPtr& goal)
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
			state_ = read_puck_states::IDLE;
			setVelocity(0, 0, 0);
			publishVelocity();
			return;
		}
		controlLoop();
		if(state_ == read_puck_states::FINISHED)
		{
			state_ = read_puck_states::IDLE;
			setVelocity(0, 0, 0);
			publishVelocity();
			result_.goal_achieved = true;
			result_.message = "Goal achieved with success!!!";
			server_.setSucceeded(result_);
			ROS_INFO("%s goal reached!!!", name_.c_str());
			return;
		}
		else if (state_ == read_puck_states::LOST)
		{
			state_ = read_puck_states::IDLE;
			setVelocity(0, 0, 0);
			publishVelocity();
			result_.goal_achieved = false;
			result_.message = "Robotino got lost!!!";
			server_.setAborted(result_, result_.message);
			ROS_ERROR("%s", result_.message.c_str());
			return;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	state_ = read_puck_states::IDLE;
	setVelocity(0, 0, 0);
	publishVelocity();
	result_.goal_achieved = false;
	result_.message = "Aborting on the goal because the node has been killed!!!";
	server_.setAborted(result_, result_.message);
}

/**
 *
 */
bool ReadPuckServer::validateNewGoal(const robotino_motion::ReadPuckGoalConstPtr& goal)
{
	if(state_ == read_puck_states::UNINITIALIZED)
	{
		result_.goal_achieved = false;
		result_.message = "Odometry not initialized yet!!!";
		server_.setAborted(result_, result_.message);
		ROS_ERROR("%s", result_.message.c_str());
		return false;
	}
	valid_colors_.clear();
	for (int i = 0; i < goal->valid_colors.size(); i++)
	{
		int counter = 0;
		Color color = Colors::toColor(goal->valid_colors[i]);
		if (color == colors::NONE)
		{	
			result_.goal_achieved = false;
			result_.message = "Invalid color code!!!";
			server_.setAborted(result_, result_.message);
			ROS_ERROR("Invalid color code: %d!!!", color);
			return false;
		}
		if (color != colors::YELLOW && color != colors::BLUE && color != colors::GREEN && color != colors::RED)
		{
			result_.goal_achieved = false;
			result_.message = Colors::toString(color) + " color is not working because it was not calibrated yet in robotino_vision package!!!";
			server_.setAborted(result_, result_.message);
			ROS_ERROR("%s", result_.message.c_str());
			return false;
		}
		/*robotino_vision::FindObjects srv;
		srv.request.color = Colors::toCode(color);
		if (find_objects_cli_.call(srv))
		{
			find_objects_cli_.
		}*/
		valid_colors_.push_back(color);
		for (int j = 0; j < valid_colors_.size(); j++)
		{
			if (color == valid_colors_[j])
			{
				counter++;
				if (counter > 1)
				{
					valid_colors_.erase(valid_colors_.begin() + j);
					break;
				}
			}
		}
	}
	if(valid_colors_.empty())
	{
		result_.goal_achieved = false;
		result_.message = "There is no valid color setted!!!";
		server_.setAborted(result_, result_.message);
		ROS_ERROR("%s", result_.message.c_str());
		return false;
	}
	verify_markers_ = goal->verify_markers;
	/*robotino_vision::FindObjects srv;
	srv.request.color = Colors::toCode(color_);
	if (!find_objects_cli_.call(srv))
	{	
		result_.goal_achieved = false;
		result_.message = Colors::toString(color_) + " puck not found!!!";
		server_.setAborted(result_, result_.message);
		ROS_ERROR("%s", result_.message.c_str());
		return false;
	}*/
	reading_start_ = ros::Time::now();
	final_color_index_ = -1;
	colors_.clear();
	number_of_markers_.clear();
	counters_.clear();
	percentage_ = 0;
	resetOdometry();
	//state_ = read_puck_states::ALIGNING_LATERAL;
	ROS_INFO("Goal accepted, reading puck!!!");
	return true;
}

/**
 *
 */
void ReadPuckServer::publishFeedback()
{
	ROS_DEBUG("%s (%f)", ReadPuckStates::toString(state_).c_str(), percentage_);
	feedback_.percentage = percentage_;
	feedback_.state = ReadPuckStates::toString(state_);
	server_.publishFeedback(feedback_);
}

/**
 *
 */
void ReadPuckServer::readParameters()
{
	
}
