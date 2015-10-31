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
	server_(nh, "read_puck", boost::bind(&ReadPuckServer::executeCallback, this, _1), false),
	align_client_("align", true)
{
	readParameters();
	find_objects_cli_ = nh_.serviceClient<robotino_vision::FindObjects>("find_objects");

	state_ = read_puck_states::UNINITIALIZED;
	percentage_ = 0;

	verify_markers_ = false;
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
	double vel_x = 0, vel_y = 0, vel_phi = 0;
	ROS_INFO("%s", ReadPuckStates::toString(state_).c_str());
	double percentage = 0;

	if (state_ == read_puck_states::ALIGNING_FRONTAL)
	{
/////////////////////////////ALINHAR COM A FRENTE NEM SEMPRE É BOM
		robotino_motion::AlignGoal frontal_alignment;
		frontal_alignment.alignment_mode = 8; // FRONT_LASER alignment mode
		frontal_alignment.distance_mode = 1; // NORMAL distance mode
		align_client_.sendGoal(frontal_alignment);
		align_client_.waitForResult(ros::Duration(1.0));
		state_ = read_puck_states::HEADING_TOWARD_PUCK;
	}
	else if (state_ == read_puck_states::HEADING_TOWARD_PUCK || state_ == read_puck_states::HEADING_BACKWARD_PUCK)
	{
		if (state_ == read_puck_states::HEADING_TOWARD_PUCK)
		{
			vel_x = .1;
		}
		else
		{
			vel_x = -.1;
		}
		robotino_vision::FindObjects srv;
		srv.request.specific_number_of_markers = 0;
		if (verify_markers_)
		{
			srv.request.verify_markers = true;
		}
		else
		{
			srv.request.verify_markers = false;
		}	
		for (int i = 0; i < valid_colors_.size(); i++)
		{
			srv.request.color = valid_colors_[i];
			find_objects_cli_.waitForExistence();
			Color color = Colors::toColor(valid_colors_[i]);
			if (!find_objects_cli_.call(srv))
			{	
				ROS_WARN("There is no %s object", Colors::toString(color).c_str());
				continue;
			}
			std::vector<int> number_of_markers;	
			for (int j = 0; j < srv.response.number_of_markers.size(); j++)
			{
				number_of_markers.push_back(srv.response.number_of_markers[i]);
			}
			if (!number_of_markers.empty())
			{
				for (int j = 0; j < number_of_markers.size(); j++)
				{
					Puck puck;
					puck.color = color;
					puck.number_of_markers = number_of_markers[j];
					updatePucks(puck);		
				}
			}
		}
	}
	if ((ros::Time::now() - reading_start_).toSec() > READING_DEADLINE) // 100%
	{
		if (state_ == read_puck_states::HEADING_TOWARD_PUCK)
		{
			reading_start_ = ros::Time::now();
			state_ = read_puck_states::HEADING_BACKWARD_PUCK;
		}
		else 
		{
			vel_x = 0;
			if (state_ == read_puck_states::HEADING_TOWARD_PUCK && pucks_.empty())
			{
				result_.goal_achieved = false;
				result_.message = "No puck!!!";
				server_.setAborted(result_, result_.message);
				ROS_ERROR("%s", result_.message.c_str());
				return;
			}
			int sum_parts = 0;
			int sum_total = 0;
			for (int i = 0; i < pucks_.size(); i++)
			{
				ROS_INFO("Index: %d, Counter: %d", i, pucks_[i].counter);
				sum_parts += i * pucks_[i].counter;
				sum_total += pucks_[i].counter;
			}
			int index = round(sum_parts / sum_total);
			result_.color = Colors::toCode(pucks_[index].color);
			result_.number_of_markers = pucks_[index].number_of_markers;
			ROS_INFO("----------------------------");
			ROS_INFO("Final Puck:/n Color:%s, Number_of_markers: %d", Colors::toString(pucks_[index].color).c_str(), pucks_[index].number_of_markers);
			state_ = read_puck_states::FINISHED;
			percentage_ = 100;

		}
	}
	if (percentage > percentage_)
	{
		percentage_ = percentage;
	}
	setVelocity(vel_x, vel_y, vel_phi);
	publishVelocity();
}

/**
 *
 */
void ReadPuckServer::updatePucks(Puck puck)
{
	for (int i = 0; i < pucks_.size(); i++)
	{
		if (puck.color == pucks_[i].color && puck.number_of_markers == pucks_[i].number_of_markers)
		{
			pucks_[i].counter++;
			return;
		}
	}
	puck.counter = 1;
	pucks_.push_back(puck);
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
	ROS_INFO("Verify markers: %s, %d", verify_markers_?"true":"false", verify_markers_);
	for (int i = 0; i < goal->valid_number_of_markers.size(); i++)
	{
		valid_number_of_markers_.push_back(goal->valid_number_of_markers[i]);
	}
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
	percentage_ = 0;
	resetOdometry();
	state_ = read_puck_states::HEADING_TOWARD_PUCK;
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
