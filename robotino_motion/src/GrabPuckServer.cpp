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
	readParameters();
	digital_readings_sub_ = nh_.subscribe("digital_readings", 1, &GrabPuckServer::digitalReadingsCallback, this);
	find_objects_cli_ = nh_.serviceClient<robotino_vision::FindObjects>("find_objects");

	state_ = grabPuckStates::UNINITIALIZED;
	percentage_ = 0;

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
	return state_ != grabPuckStates::UNINITIALIZED && state_ != grabPuckStates::IDLE;
}

/**
 *
 */
void GrabPuckServer::start()
{
	if (state_ == grabPuckStates::UNINITIALIZED)
	{
		server_.start();
		state_ = grabPuckStates::IDLE;
	}
}

/**
 *
 */
void GrabPuckServer::stop()
{
	setVelocity(0, 0, 0);
	publishVelocity();
	state_ = grabPuckStates::IDLE;
	result_.goal_achieved = false;
	result_.message = "Unexpected emergency stop request!!!";
	server_.setAborted(result_, result_.message);
}

/**
 *
 */
void GrabPuckServer::controlLoop()
{
	double vel_x = 0, vel_y = 0, vel_phi = 0;
	ROS_DEBUG("%s", GrabPuckStates::toString(state_).c_str());
	if (!is_loaded_ || nframes_no_puck_ > 50)
	{
		robotino_vision::FindObjects srv;
		srv.request.color = Colors::toProduct(color_);
		if (!find_objects_cli_.call(srv))
		{	
			/*ROS_ERROR("Puck not found!!!");
			state_ = grabPuckStates::LOST;*/
			return;
		}

		std::vector<float> distances = srv.response.distances;
		std::vector<float> directions = srv.response.directions; 
		
		int num_products = srv.response.distances.size();
		if (num_products > 0 && state_ != grabPuckStates::GRABBING_PUCK)
		{
			nframes_no_puck_ = 0;
		
			int closest_index = 0;
			for (int i = 0; i < num_products; i++)
			{
				if (distances.at(i) < distances.at(closest_index))
				{
					closest_index = i;
				}
			}

			double max_error_lateral = 50, max_error_frontal = 40;
			double error_lateral = directions.at(closest_index);
			if (error_lateral > max_error_lateral)
			{
				 error_lateral = max_error_lateral;
			}
			double error_frontal = distances.at(closest_index);
			if (error_frontal > max_error_frontal)
			{
				 error_frontal = max_error_frontal;
			}

			float tolerance_lateral = 0.1, tolerance_frontal = 35;
			double K_error_lateral = .3, K_error_frontal = .002;
			double percentage_f, percentage_0, tolerance, max_error, error;
			if (fabs(error_lateral) > tolerance_lateral) // 0% a 50%
			{
				vel_x = 0;
				vel_y = -K_error_lateral * error_lateral;
				vel_phi = 0;
				percentage_0 = 0;
				percentage_f = 50;
				tolerance = tolerance_lateral;
				max_error = max_error_lateral;
				error = fabs(error_lateral);
			}
			else if (fabs(error_frontal) > tolerance_frontal) // 51% a 70%
			{
				vel_x = K_error_frontal * error_frontal;
				vel_y = 0;
				vel_phi = 0;
				percentage_0 = 51;
				percentage_f = 70;
				tolerance = tolerance_frontal;
				max_error = max_error_frontal;
				error = fabs(error_frontal);
			}
			else //71% a 90%
			{
				state_ = grabPuckStates::GRABBING_PUCK;
				vel_x = .05;
				vel_y = 0;
				vel_phi = 0;
			}
			percentage_ = percentage_0 + percentage_f * (max_error - error) / (max_error - tolerance);
		}
		else if (!is_loaded_ && state_ == grabPuckStates::GRABBING_PUCK) //91% a 99%
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
	else // 100%
	{	
		percentage_ = 100;
		state_ = grabPuckStates::FINISHED;
	}
	setVelocity(vel_x, vel_y, vel_phi);
	publishVelocity();
	publishFeedback();
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
			state_ = grabPuckStates::IDLE;
			setVelocity(0, 0, 0);
			publishVelocity();
			return;
		}
		controlLoop();
		if(state_ == grabPuckStates::FINISHED)
		{
			state_ = grabPuckStates::IDLE;
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
	state_ = grabPuckStates::IDLE;
	setVelocity(0, 0, 0);
	publishVelocity();
	result_.goal_achieved = false;
	result_.message = "Aborting on the goal because the node has been killed!!!";
	server_.setAborted(result_, result_.message);
}

/**
 *
 */
bool GrabPuckServer::validateNewGoal(const robotino_motion::GrabPuckGoalConstPtr& goal)
{
	if(state_ == grabPuckStates::UNINITIALIZED)
	{
		result_.goal_achieved = false;
		result_.message = "Odometry not initialized yet!!!";
		server_.setAborted(result_, result_.message);
		ROS_ERROR("%s", result_.message.c_str());
		return false;
	}
	color_ = Colors::convertProductToColor(goal->color);
	if (color_ == colors::NONE)
	{	
		result_.goal_achieved = false;
		result_.message = "Invalid color code!!!";
		server_.setAborted(result_, result_.message);
		ROS_ERROR("Invalid color code: %d!!!", goal->color);
		return false;
	}
	robotino_vision::FindObjects srv;
	srv.request.color = Colors::toProduct(color_);
	if (!find_objects_cli_.call(srv))
	{	
		result_.goal_achieved = false;
		result_.message = Colors::toString(color_) + " puck not found!!!";
		server_.setAborted(result_, result_.message);
		ROS_ERROR("%s", result_.message.c_str());
		//state_ = grabPuckStates::LOST;
		return false;
	}
	percentage_ = 0;
	state_ = grabPuckStates::FINDING_PUCK;
	ROS_INFO("Goal accepted, grabbing %s puck!!!", Colors::toString(color_).c_str());
	return true;
}

/**
 *
 */
void GrabPuckServer::publishFeedback()
{
	feedback_.percentage = percentage_;
	feedback_.state = GrabPuckStates::toString(state_);
	server_.publishFeedback(feedback_);
}


/**
 *
 */
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
