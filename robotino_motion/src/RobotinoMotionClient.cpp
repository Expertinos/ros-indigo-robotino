
/*
 * RobotinoMotionClient.cpp
 *
 *  Created on: 14.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#include "RobotinoMotionClient.h"

RobotinoMotionClient::RobotinoMotionClient():
	client_( "motion", false),
	max_time_( 1.0 )
{
	goal_sub_ = nh_.subscribe( "goal", 1,
			&RobotinoMotionClient::goalCallback, this );
}

RobotinoMotionClient::~RobotinoMotionClient()
{
	goal_sub_.shutdown();
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

	ROS_INFO( "Sending goal (move_x[m], move_y[m], move_phi[rad], movement_type, task_type, interruption_condition, alignment_device) = (%f, %f, %f, %d, %d, %d, %d)",  goal.move_x, goal.move_y, goal.move_phi, goal.movement_type, goal.task_type, goal.interruption_condition, goal.alignment_device);

	if(checkServer())
	{
		sendGoal(goal);
	}
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
	client_.sendGoal( goal );
	ROS_INFO("Goal sent");
}
