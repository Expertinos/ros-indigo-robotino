
/*
 * RobotinoMotionClient.cpp
 *
 *  Created on: 2014
 *      Author: expertinos.unifei@gmail.com
 */

#include "RobotinoMotionClient.h"

RobotinoMotionClient::RobotinoMotionClient():
	client_("motion", false),
	max_time_( 1.0 )
{
	goal_sub_ = nh_.subscribe( "goal", 1,
			&RobotinoMotionClient::goalCallback, this );
	has_arrived_pub_ = nh_.advertise<robotino_motion::HasArrived>("has_arrived", 1, true );

	move_srv_ = nh_.advertiseService("move_to", &RobotinoMotionClient::moveTo, this);

	get_map_cli_ = nh_.serviceClient<robotino_mapping::GetMap>("get_map");

	set_achieved_goal_srv_ = nh_.advertiseService("set_achieved_goal", &RobotinoMotionClient::setAchievedGoal, this);

	achieved_goal_srv_ = nh_.advertiseService("achieved_goal", &RobotinoMotionClient::achievedGoal, this);

	odometry_sub_cli_ = nh_.subscribe( "odom", 1, &RobotinoMotionClient::odomCallback, this );

	curr_x_ = (0.0);
	curr_y_ = (0.0);
	achieved_goal_ = false;
}

RobotinoMotionClient::~RobotinoMotionClient()
{
	goal_sub_.shutdown();
	has_arrived_pub_.shutdown();
	move_srv_.shutdown();
	get_map_cli_.shutdown();
	achieved_goal_srv_.shutdown();
	odometry_sub_cli_.shutdown();
	set_achieved_goal_srv_.shutdown();
}

bool RobotinoMotionClient::moveTo(robotino_motion::MoveTo::Request &req, robotino_motion::MoveTo::Response &res)
{
/*	int xA, yA, xB, yB;

	robotino_mapping::GetMap service;
	int area = req.obstacle_code;
	service.request.area = area;
	if (!get_map_cli_.call(service))
	{
		ROS_ERROR("Map not found!!!");
		return false;
	}
	int size = service.response.map.size();
	ROS_INFO("size %d", size);
	vector<int> vet;
	for (int i = 0; i < size; i++)
	{
		vet.push_back(service.response.map[i]);
		ROS_INFO("VET[%d] = %d", i, vet[i]);
	}
	
	xA = curr_x_;
	yA = curr_y_;
	xB = req.x;
	yB = req.y;

	node node;
	for (int i = 0; i < 17; i++)
	{
		for (int j = 0; j < 17; j++)
			if (vet[1 * 17 + j] == 1)
				std::cout << " . ";
			else 
				std::cout << "   ";
		std::cout << endl;
	}
	node.findpath(vet, yA, xA, yB, xB);
	
	return true;*/
	robotino_motion::MotionGoal goal1 = buildGoal(1.0, 0.0, 0.0, 0, 0, 0, 0);
	sendGoal(goal1);
	
}

bool RobotinoMotionClient::achievedGoal(robotino_motion::AchievedGoal::Request &req, robotino_motion::AchievedGoal::Response &res)
{
	res.achieved_goal = achieved_goal_;
	return true;
}

bool RobotinoMotionClient::setAchievedGoal(robotino_motion::SetAchievedGoal::Request &req, robotino_motion::SetAchievedGoal::Response &res)
{
	achieved_goal_ = req.achieved_goal;
	res.succeed = true;
	return true;
}

void RobotinoMotionClient::odomCallback( const nav_msgs::OdometryConstPtr& msg )
{
	curr_x_ = msg->pose.pose.position.x;
	curr_y_ = msg->pose.pose.position.y;
}

robotino_motion::MotionGoal RobotinoMotionClient::buildGoal(float move_x, float move_y, float move_phi, int movement_type, int task_type, int interruption_condition, int alignment_device)
{
	robotino_motion::MotionGoal goal;
	goal.move_x = move_x;
	goal.move_y = move_y;
	goal.move_phi = move_phi;
	goal.movement_type = movement_type;
	goal.task_type = task_type;
	goal.interruption_condition = interruption_condition;
	goal.alignment_device = alignment_device;

	/*ROS_INFO( "Pushing goal (move_x[m], move_y[m], move_phi[rad], "
			"movement_type, task_type, interruption_condition, alignment_device) = "
			"(%f, %f, %f, %d, %d, %d, %d)",
			goal.move_x, goal.move_y, goal.move_phi, goal.movement_type, goal.task_type,
			goal.interruption_condition, goal.alignment_device);*/
	return goal;
}
	

void RobotinoMotionClient::goalCallback( const robotino_motion::MotionGoalConstPtr& msg )
{
	robotino_motion::MotionGoal goal;
	goal.move_x = msg->move_x;
	goal.move_y = msg->move_y;
	goal.move_phi = msg->move_phi;
	goal.movement_type = msg->movement_type;
	goal.task_type = msg->task_type;
	goal.interruption_condition = msg->interruption_condition;
	goal.alignment_device = msg->alignment_device;

	/*ROS_INFO( "Pushing goal (move_x[m], move_y[m], move_phi[rad], "
			"movement_type, task_type, interruption_condition, alignment_device) = "
			"(%f, %f, %f, %d, %d, %d, %d)",
			goal.move_x, goal.move_y, goal.move_phi, goal.movement_type, goal.task_type,
			goal.interruption_condition, goal.alignment_device);*/

	/*if(goal.alignment_device == 0)
	{
		queue_.push(goal);
		//ROS_INFO("%d", queue_.size());
	}else
	{
		if(checkServer())
		{
			popGoalCallback(queue_);
		}
	}*/
	sendGoal(goal);
	achieved_goal_ = false;
}

void RobotinoMotionClient::popGoalCallback( const std::queue<robotino_motion::MotionGoal> )
{
	robotino_motion::MotionGoal goal;

	if(!queue_.empty())
	{
		goal = queue_.front();
		queue_.pop();
		//std::cout << goal;
		//ROS_INFO("%d", queue_.size());
		sendGoal(goal);
	}
	/*ROS_INFO( "Pushing goal (move_x[m], move_y[m], move_phi[rad], movement_type, task_type, interruption_condition, alignment_device) = (%f, %f, %f, %d, %d, %d, %d)", goal.move_x, goal.move_y, goal.move_phi, goal.movement_type, goal.task_type, goal.interruption_condition, goal.alignment_device);*/
}

void RobotinoMotionClient::doneCallBack( const actionlib::SimpleClientGoalState& state,
			const robotino_motion::MotionResultConstPtr& result)
{
	ROS_INFO("Finished in state [%s]", state.toString().c_str());
	ROS_INFO("Answer: %i", result->achieved_goal);
	achieved_goal_ = true;
	//popGoalCallback(queue_);
}

bool RobotinoMotionClient::checkServer()
{
	for( int i = 0; i < 5; ++i)
	{
		ros::spinOnce();
		if( client_.waitForServer( ros::Duration( 1.0 ) ) )
		{
			ROS_INFO( "Connected to the motion server" );
			return true;
		}
		else
		{
			ROS_INFO( "Waiting for motion server" );
		}
	}

	ROS_ERROR( "motion server not running" );
	return false;
}

void RobotinoMotionClient::spin()
{
	ros::Rate loop_rate ( 5 );
	ros::Time start_time = ros::Time::now();

	while( nh_.ok() )
	{
		if( client_.waitForResult( ros::Duration( 1.0 ) ) )
		{
			ROS_INFO("Motion succeeded");
			break;
		}
		else
		{
			ROS_INFO("Motion is being executed");
		}

		if( ( ros::Time::now() - start_time ).toSec() > max_time_ )
		{
			ROS_INFO( "Timeout: Aborting Motion" );
			client_.cancelAllGoals();
			break;
		}

		robotino_motion::HasArrived has_arrived;
		has_arrived.has_arrived = queue_.empty();

		has_arrived_pub_.publish(has_arrived);

		ros::spinOnce();
		loop_rate.sleep();
	}
}

void RobotinoMotionClient::setMaxTime( const float& time )
{
	max_time_ = time;
}

void RobotinoMotionClient::sendGoal( const robotino_motion::MotionGoal& goal )
{
	client_.sendGoal( goal , boost::bind(&RobotinoMotionClient::doneCallBack, this, _1, _2), Client::SimpleActiveCallback(), Client::SimpleFeedbackCallback());

	ROS_INFO("Goal sent");
}
