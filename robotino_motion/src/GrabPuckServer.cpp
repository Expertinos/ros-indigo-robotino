/**
 *  GrabPuckServer.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 01/10/2015
 *  Modified on: ??/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "GrabPuckServer.h"

/**
 *
 */
GrabPuckServer::GrabPuckServer(ros::NodeHandle nh, std::string ns) : 
	Server(nh, "Grab Puck", ns),
	server_(nh, "grab_puck", boost::bind(&GrabPuckServer::executeCallback, this, _1), false)
{
	server_.registerPreemptCallback(boost::bind(&GrabPuckServer::preemptCallback, this));
	readParameters();
	digital_readings_sub_ = nh_.subscribe("digital_readings", 1, &GrabPuckServer::digitalReadingsCallback, this);
	find_objects_cli_ = nh_.serviceClient<robotino_vision::FindObjects>("find_objects");

	state_ = GrabPuckStates::UNINITIALIZED;
	is_loaded_ = false;
	nframes_no_puck_ = 0;
}

/**
 *
 */
GrabPuckServer::~GrabPuckServer() 
{
	digital_readings_sub_.shutdown();
	find_objects_cli_.shutdown();
	server_.shutdown();
}

/**
 *
 */
bool GrabPuckServer::isActing()
{
	return state_ != GrabPuckStates::UNINITIALIZED && state_ != GrabPuckStates::IDLE;
}

/**
 *
 */
void GrabPuckServer::start()
{
	if (state_ == GrabPuckStates::UNINITIALIZED)
	{
		server_.start();
		state_ = GrabPuckStates::IDLE;
	}
}

/**
 *
 */
void GrabPuckServer::stop()
{
	setVelocity(0, 0, 0);
	publishVelocity();
	state_ = GrabPuckStates::IDLE;
	result_.goal_achieved = false;
	server_.setAborted(result_, "Emergency stop requested!!!");
}

/**
 *
 */
void GrabPuckServer::controlLoop()
{
	//ROS_INFO("no puck=%d", nframes_no_puck_);
	double vel_x = 0, vel_y = 0, vel_phi = 0;
	if (!is_loaded_ || nframes_no_puck_ > 50)
	{
		robotino_vision::FindObjects srv;
		srv.request.color = Colors::toProduct(color_);
		//ROS_INFO("Product: %s", Colors::convertProductToString(srv.request.color).c_str());
		if (!find_objects_cli_.call(srv))
		{	
			ROS_ERROR("Puck not found!!!");
			state_ = GrabPuckStates::LOST;
			return;
		}

		std::vector<float> distances,  directions;
		distances = srv.response.distances;
		directions = srv.response.directions; 
		
		int closest_index = 0;
		int num_products = srv.response.distances.size();
		if (num_products > 0 && state_ != GrabPuckStates::GRABING_PUCK)
		{
			nframes_no_puck_ = 0;
		
			for (int i = 0; i < num_products; i++)
			{
				if (distances.at(i) < distances.at(closest_index))
				{
					closest_index = i;
				}
			}

			double max_error_front = 40;
			double error_front = 0;
			error_front = distances.at(closest_index);
		
			if (error_front > max_error_front)
			{
				 error_front = max_error_front;
			}

			float tolerance_y = 0.1, tolerance_x = 35;
			double error_abs = fabs(directions.at(closest_index));
			double K_error = .3, K_error_front = .002;
		
			if (directions.at(closest_index) < -tolerance_y)
			{
				vel_x = 0;
				vel_y = K_error * error_abs;
				vel_phi = 0;
			}
			else if (directions.at(closest_index) > tolerance_y)
			{
				vel_x = 0;
				vel_y = -K_error * error_abs;
				vel_phi = 0;
			}
			else if (error_front > tolerance_x)
			{
				vel_x = K_error_front * error_front;
				vel_y = 0;
				vel_phi = 0;
			}
			else
			{
				state_ = GrabPuckStates::GRABING_PUCK;
				vel_x = .05;
				vel_y = 0;
				vel_phi = 0;
			}
		}
		else if (!is_loaded_ && state_ != GrabPuckStates::GRABING_PUCK) 
		{
			vel_x = .05;
			vel_y = 0;
			vel_phi = 0;
		}
		else 
		{
			nframes_no_puck_++;
		}			
	}
	else 
	{	
		state_ = GrabPuckStates::IDLE;
	}
	setVelocity(vel_x, vel_y, vel_phi);
	publishVelocity();
}

/**
 *
 */
void GrabPuckServer::executeCallback(const robotino_motion::GrabPuckGoalConstPtr& goal)
{
	ros::Rate loop_rate(20);
	if(!validateNewGoal(goal))
	{
		ROS_WARN("Goal not accepted!!!");
		result_.goal_achieved = false;
		server_.setAborted(result_, "No puck found or invalid color code!!!");
		return;
	}
	Color color = Colors::convertProductToColor(goal->color);
	ROS_DEBUG("Color desired to grab: %s", Colors::convertProductToString(color).c_str());
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
			state_ = GrabPuckStates::IDLE;
			setVelocity(0, 0, 0);
			publishVelocity();
			return;
		}
		controlLoop();
		if(state_ == GrabPuckStates::FINISHED)
		{
			state_ = GrabPuckStates::IDLE;
			setVelocity(0, 0, 0);
			publishVelocity();
			result_.goal_achieved = true;
			server_.setSucceeded(result_);
			return;
		}
		if(state_ != GrabPuckStates::IDLE)
		{
			publishVelocity();
			server_.publishFeedback(feedback_);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	state_ = GrabPuckStates::IDLE;
	setVelocity(0, 0, 0);
	publishVelocity();
	result_.goal_achieved = false;
	server_.setAborted(result_, "Aborting on the goal because the node has been killed");
}


/**
 *
 */
void GrabPuckServer::preemptCallback()
{
	color_ = NONE;
	state_ = GrabPuckStates::IDLE;
}

/**
 *
 */
bool GrabPuckServer::validateNewGoal(const robotino_motion::GrabPuckGoalConstPtr& goal)
{
	if(state_ == GrabPuckStates::UNINITIALIZED)
	{
		ROS_ERROR("Odometry not initialized!!!");
		return false;
	}

	ROS_DEBUG("Accepting new goal!!!");
	int color_code = goal->color;
	if (color_code == -1)
	{
		ROS_WARN("Invalid color code: %d!!!", color_code);
		return false;
	}
	goal_.color = color_code;
	color_ = Colors::convertProductToColor(color_code);
	
	robotino_vision::FindObjects srv;
	srv.request.color = Colors::toProduct(color_);
	if (!find_objects_cli_.call(srv))
	{	
		ROS_ERROR("%s Puck not found!!!", Colors::toString(color_).c_str());
		state_ = GrabPuckStates::LOST;
		return false;
	}
	ROS_INFO("Goal accepted, grabbing %s puck!!!", Colors::toString(color_).c_str());
	return true;
}

void GrabPuckServer::digitalReadingsCallback(const robotino_msgs::DigitalReadings& msg)
{
	is_loaded_ = !msg.values.at(0);
}

/**
 *
 */
void GrabPuckServer::readParameters()
{
	
}
