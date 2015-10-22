/**
 *  StorePuckServer.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 01/10/2015
 *  Modified on: ??/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "StorePuckServer.h"

/**
 *
 */
StorePuckServer::StorePuckServer(ros::NodeHandle nh) : 
	Server(nh, "Store Puck"),
	server_(nh, "store_puck", boost::bind(&StorePuckServer::executeCallback, this, _1), false),
	align_client_("align", true)
{
	readParameters();
	digital_readings_sub_ = nh_.subscribe("digital_readings", 1, &StorePuckServer::digitalReadingsCallback, this);	
	distance_sensors_sub_ = nh_.subscribe("distance_sensors", 1, &StorePuckServer::distanceSensorsCallback, this);
	laser_scan_sub_ = nh_.subscribe("scan", 10, &StorePuckServer::laserScanCallback, this);
	find_areas_cli_ = nh_.serviceClient<robotino_vision::FindInsulatingTapeAreas>("find_areas");

	state_ = store_puck_states::UNINITIALIZED;
	percentage_ = 0;

	loaded_ = false;

	left_index_ = 0;
	right_index_ = 0;
	lateral_ = false;

	laser_front_ = 0.0;
	laser_right_ = 0.0;
	laser_left_ = 0.0;
	phi_fut_ = 0.0;
}

/**
 *
 */
StorePuckServer::~StorePuckServer() 
{
	digital_readings_sub_.shutdown();
	laser_scan_sub_.shutdown();
	find_areas_cli_.shutdown();
	server_.shutdown();
}

/**
 *
 */
bool StorePuckServer::isActing()
{
	return state_ != store_puck_states::UNINITIALIZED && state_ != store_puck_states::IDLE;
}

/**
 *
 */
void StorePuckServer::start()
{
	if (!find_areas_cli_.exists())
	{
		ROS_WARN("Waiting for robotino_vision_node!!!");
	}
	find_areas_cli_.waitForExistence();
	ROS_INFO("/find_areas service server is running!!!");
	if(!align_client_.isServerConnected())
	{
		ROS_INFO("Esperando align server node!!");
	}
	align_client_.waitForServer();
	ROS_INFO("/align action server is running!!!");
	if (state_ == store_puck_states::UNINITIALIZED)
	{
		server_.start();
		state_ = store_puck_states::IDLE;
	}
}

/**
 *
 */
void StorePuckServer::stop()
{
	setVelocity(0, 0, 0);
	publishVelocity();
	state_ = store_puck_states::IDLE;
	result_.goal_achieved = false;
	result_.message = "Unexpected emergency stop request!!!";
	server_.setAborted(result_, result_.message);
}

/**
 *
 */
void StorePuckServer::controlLoop()
{
	double vel_x = 0, vel_y = 0, vel_phi = 0;
	ROS_DEBUG("%s", StorePuckStates::toString(state_).c_str());
	double percentage = 0;

	switch (mode_)
	{
		case store_modes::LASER_SCAN:
			robotino_motion::AlignGoal goal;
			goal.alignment_mode = 8; //alignment_modes::LASER_FRONT; //front align
			goal.distance_mode = 1;
			align_client_.sendGoal(goal);
			align_client_.waitForResult();

			resetOdometry();
			while(getOdometry_PHI() < M_PI/2)
			{
				setVelocity(0, 0, 0.5);
				publishVelocity();
			}
			std::cout<<"getOdometry_PHI: "<<getOdometry_PHI()<<std::endl;
			setVelocity(0, 0, 0);
			publishVelocity();

			goal.alignment_mode = 9; //AlignmentModes::toCode(alignment_modes::LASER_RIGHT_LEFT)); //lateral align
			goal.distance_mode = 1;
			align_client_.sendGoal(goal);
			align_client_.waitForResult();

			goal.alignment_mode = 8;
			goal.distance_mode = 1;
			align_client_.sendGoal(goal);
			align_client_.waitForResult();

			goal.alignment_mode = 9; //lateral align
			goal.distance_mode = 1;
			align_client_.sendGoal(goal);
			align_client_.waitForResult();

			while(laser_front_ > 0.33)
			{
				setVelocity(0.1, 0, 0);
				publishVelocity();
			}
			setVelocity(0, 0, 0);
			publishVelocity();
			while(laser_front_ < 1.0)
			{
				setVelocity(-0.1, 0, 0);
				publishVelocity();
			}
			setVelocity(0, 0, 0);
			publishVelocity();

			if(laser_right_ < 0.7 || laser_left_ < 0.7)
			{
				while(laser_right_ < 0.7)
				{
					setVelocity(0, 0.1, 0);
					publishVelocity();
				}
				setVelocity(0, 0, 0);
				publishVelocity();
				while(laser_left_ < 0.7)
				{
					setVelocity(0, -0.1, 0);
					publishVelocity();
				}
				setVelocity(0, 0, 0);
				publishVelocity();
			}
			break;
		/*case store_modes::VISION:
			///
			break;
		default:
			ROS_ERROR("Storage Mode not supported yet!!!");
			return;*/
	}



	/*if (!loaded_)
	{
		robotino_vision::FindObjects srv;
		srv.request.color = Colors::toCode(color_);
		if (!find_objects_cli_.call(srv))
		{	
			ROS_ERROR("Puck not found!!!");
			state_ = store_puck_states::LOST;
			return;
		}

		std::vector<float> distances = srv.response.distances;
		std::vector<float> directions = srv.response.directions; 
		int num_products = srv.response.distances.size();
		if (num_products > 0 && state_ != store_puck_states::GRABBING_PUCK)
		{
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
			double K_error_lateral = 0.3, K_error_frontal = 0.005;
			double percentage_f, percentage_0, tolerance, max_error, error;
			if (fabs(error_lateral) > tolerance_lateral) // 0% a 49%
			{
				state_ = store_puck_states::ALIGNING_LATERAL;
				vel_y = -K_error_lateral * error_lateral;
				percentage_0 = 0;
				percentage_f = 49;
				tolerance = tolerance_lateral;
				max_error = max_error_lateral;
				error = fabs(error_lateral);
			}
			else if (fabs(error_frontal) > tolerance_frontal) // 50% a 69%
			{
				state_ = store_puck_states::HEADING_TOWARD_PUCK;
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
				state_ = store_puck_states::GRABBING_PUCK;
				storebing_start_ = ros::Time::now();
			}
			percentage = percentage_0 + (percentage_f - percentage_0) * (max_error - error) / (max_error - tolerance);
		}
		else if (state_ == store_puck_states::GRABBING_PUCK) // 70% a 79%
		{
			state_ = store_puck_states::GRABBING_PUCK;
			vel_x = .14;
			double percentage_0 = 70, percentage_f = 79;
			double elapsed_time = (ros::Time::now() - storebing_start_).toSec();
			if (elapsed_time > GRABBING_DEADLINE)
			{
				state_ = store_puck_states::LOST;
				ROS_ERROR("Storebing state deadline expired!!!");
			}
			percentage = percentage_0 + (percentage_f - percentage_0) * elapsed_time / GRABBING_DEADLINE;
		}		
	}
	else 
	{		
		if (state_ == store_puck_states::GRABBING_PUCK)
		{
			delta_x_ = -getOdometry_X();
			resetOdometry();
			state_ = store_puck_states::ROTATING;
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
		double K_error_linear = 1.2, K_error_angular = 1.2;
		double percentage_f, percentage_0, tolerance, max_error, error;
		if (fabs(error_angular) > tolerance_angular) // 80% a 89%
		{
			state_ = store_puck_states::ROTATING;
			vel_phi = K_error_angular * error_angular;
			percentage_0 = 80;
			percentage_f = 89;
			tolerance = tolerance_angular;
			max_error = max_error_angular;
			error = fabs(error_angular);
			
		}
		else if (fabs(error_linear) > tolerance_linear) // 90% a 99%
		{
			state_ = store_puck_states::GOING_BACK_TO_ORIGIN;
			vel_x = K_error_linear * error_linear;
			percentage_0 = 91;
			percentage_f = 99;
			tolerance = tolerance_linear;
			max_error = max_error_linear;
			error = fabs(error_linear);			
		}
		percentage = percentage_0 + (percentage_f - percentage_0) * (max_error - error) / (max_error - tolerance);
	}*/
	if (vel_x == 0 && vel_y == 0 && vel_phi == 0) // 100%
	{
		state_ = store_puck_states::FINISHED;
		percentage_ = 100;
	}
	if (percentage > percentage_)
	{
		percentage_ = percentage;
	}
	setVelocity(vel_x, vel_y, vel_phi);
	publishVelocity();
	publishFeedback();
}

/**
 *
 */
void StorePuckServer::executeCallback(const robotino_motion::StorePuckGoalConstPtr& goal)
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
			state_ = store_puck_states::IDLE;
			setVelocity(0, 0, 0);
			publishVelocity();
			return;
		}
		controlLoop();
		if(state_ == store_puck_states::FINISHED)
		{
			state_ = store_puck_states::IDLE;
			setVelocity(0, 0, 0);
			publishVelocity();
			result_.goal_achieved = true;
			result_.message = "Goal achieved with success!!!";
			server_.setSucceeded(result_);
			ROS_INFO("%s goal reached!!!", name_.c_str());
			return;
		}
		else if (state_ == store_puck_states::LOST)
		{
			state_ = store_puck_states::IDLE;
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
	state_ = store_puck_states::IDLE;
	setVelocity(0, 0, 0);
	publishVelocity();
	result_.goal_achieved = false;
	result_.message = "Aborting on the goal because the node has been killed!!!";
	server_.setAborted(result_, result_.message);
}

/**
 *
 */
bool StorePuckServer::validateNewGoal(const robotino_motion::StorePuckGoalConstPtr& goal)
{
	if(state_ == store_puck_states::UNINITIALIZED)
	{
		result_.goal_achieved = false;
		result_.message = "Odometry not initialized yet!!!";
		server_.setAborted(result_, result_.message);
		ROS_ERROR("%s", result_.message.c_str());
		return false;
	}
	if(!loaded_)
	{
		result_.goal_achieved = false;
		result_.message = "Robotino is not loaded!!!";
		server_.setAborted(result_, result_.message);
		ROS_ERROR("%s", result_.message.c_str());
		return false;
	}
	mode_ = StoreModes::newInstance(goal->mode);
	if (mode_ == store_modes::NONE)
	{	
		result_.goal_achieved = false;
		result_.message = "Invalid storage mode code!!!";
		server_.setAborted(result_, result_.message);
		ROS_ERROR("Invalid storage mode code: %d!!!", goal->mode);
		return false;
	}
	// seria bom verificar se tem peça na área (se a area é muito pequena)
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
	percentage_ = 0;
	resetOdometry();
	state_ = store_puck_states::STORING_PUCK;
	ROS_INFO("Goal accepted, storing puck in %s mode!!!", StoreModes::toString(mode_).c_str());
	return true;
}

/**
 *
 */
void StorePuckServer::publishFeedback()
{
	ROS_DEBUG("%s (%f)", StorePuckStates::toString(state_).c_str(), percentage_);
	feedback_.percentage = percentage_;
	feedback_.state = StorePuckStates::toString(state_);
	server_.publishFeedback(feedback_);
}


/**
 *
 */
void StorePuckServer::digitalReadingsCallback(const robotino_msgs::DigitalReadings& msg)
{
	loaded_ = !msg.values.at(0);
}

/**
 *
 */
void StorePuckServer::distanceSensorsCallback(const sensor_msgs::PointCloud& msg)
{
	if (!lateral_)
	{
		left_ir_ = msg.points[left_index_].x; 
		right_ir_ = msg.points[right_index_].x; 
	}
	else
	{
		left_ir_ = msg.points[left_index_].y; 
		right_ir_ = msg.points[right_index_].y;
	}
}

void StorePuckServer::laserScanCallback(const sensor_msgs::LaserScan& msg)
{
	int central_point = 0;
	int extreme_point = 0;
	central_point = (msg.ranges.size() - 1) / 2;
	extreme_point = msg.ranges.size() - 1;

	laser_front_ = msg.ranges[central_point];
	laser_right_ = msg.ranges[0];
	laser_left_ = msg.ranges[extreme_point];

}

/**
 *
 */
void StorePuckServer::readParameters()
{
	
}
