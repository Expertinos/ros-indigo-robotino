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
	server_.registerGoalCallback(boost::bind(&GrabPuckServer::goalCB, this));
	server_.registerPreemptCallback(boost::bind(&GrabPuckServer::preemptCallback, this));
	readParameters();
	digital_readings_sub_ = nh_.subscribe("digital_readings", 1, &GrabPuckServer::digitalReadingsCallback, this);
	find_objects_cli_ = nh_.serviceClient<robotino_vision::FindObjects>("find_objects");

	state_ = NON_INITIALIZED;
	is_loaded_ = false;
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
	return state != NON_INITIALIZED && state_ != IDLE;
}

/**
 *
 */
void GrabPuckServer::start()
{
	if (state_ = NON_INITIALIZED)
	{
		server_.start();
	}
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
		ROS_INFO("Product: %s", Colors::getProductString(srv.request.color));
		vector<float> distances,  directions;
		if (!find_objects_cli_.call(srv))
		{	
			ROS_ERROR("Puck not found!!!");
			state_ = LOST;
			return;
		}
		int num_products = srv.response.distances.size();
		distances = srv.response.distances;
		directions = srv.response.directions; 
		
		int closest_index = 0;
		if (num_products > 0 && state_ != GRABBING)
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
				up_to_grab_puck_ = true;
				vel_x = .05;
				vel_y = 0;
				vel_phi = 0;
			}
		}
		else if (!is_loaded_ && state_ != GRABBING) 
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
		state_ = IDLE;
	}
	setVelocity(vel_x, vel_y, vel_phi);
	publishVelocity();
}

/**
 *
 */
void GrabPuckServer::executeCallback(const robotino_motion::GrabPuckGoalConstPtr& goal)
{
	Color color = Colors::convertProductToColor(goal->color);
	ROS_INFO("executeCb: desire to grab ", Colors::getProductString);
}


/**
 *
 */
void GrabPuckServer::preemptCallback()
{
	ROS_INFO("executeCb: desire to grab ", Colors::getProductString);
}

/**
 *
 */
bool GrabPuckServer::acceptNewGoal(const robotino_motion::GrabPuckGoalConstPtr& goal)
{
	ROS_INFO("Accepting new goal!!!");
	int color_code = goal->color;
	if (color == -1)
	{
		ROS_WARN("Invalid color code: %d!!!", color_code);
		return false;
	}
	goal_.color = color_code;
	color_ = Colors::convertProductToColor(color_code);
	return acceptNewGoal(goal);
}

/**
 *
 */
bool GrabPuckServer::readParameters()
{
	
}
