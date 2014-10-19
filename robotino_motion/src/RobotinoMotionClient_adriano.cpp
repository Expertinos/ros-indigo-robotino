
/*
 * RobotinoMotionClient.cpp
 *
 *  Created on: 2011
 *      Author: indorewala@servicerobotics.eu
 *	Modified on: 2014
 *		Author: expertinos.unifei@gmail.com
 */

#include "RobotinoMotionClient.h"


RobotinoMotionClient::RobotinoMotionClient():
	client_( "motion", false),
	curr_x_( 0.0 ),
	curr_y_( 0.0 ),
	curr_phi_( 0.0 ),
	prev_phi_( 0.0 ),
	dist_moved_x_( 0.0 ),
	dist_moved_y_( 0.0 ),
	dist_rotated_( 0.0 ),
	forward_goal_x_( 0.0 ),
	forward_goal_y_( 0.0 ),
	rotation_goal_( 0.0 ),
	start_x_( 0.0 ),
	start_y_( 0.0 ),
	start_phi_( 0.0 ),
	odom_set_(false )
{
	goal_sub_ = nh_.subscribe( "goal", 1,
			&RobotinoMotionClient::goalCallback, this );
	has_arrived_pub_ = nh_.advertise<robotino_motion::HasArrived>("has_arrived", 1, true );

	move_srv_ = nh_.advertiseService("move_to", &RobotinoMotionClient::moveTo, this);

	get_map_cli_ = nh_.serviceClient<robotino_mapping::GetMap>("get_map");

	odometry_sub_cli_ = nh_.subscribe( "odom", 1,
				&RobotinoMotionClient::odomCallback, this );



}

RobotinoMotionClient::~RobotinoMotionClient()
{
	goal_sub_.shutdown();
	has_arrived_pub_.shutdown();
	move_srv_.shutdown();
	get_map_cli_.shutdown();
	odometry_sub_cli_.shutdown();
}

void RobotinoMotionClient::odomCallback( const nav_msgs::OdometryConstPtr& msg )
{
	curr_x_ = msg->pose.pose.position.x;
	curr_y_ = msg->pose.pose.position.y;
	curr_phi_ = tf::getYaw( msg->pose.pose.orientation );

	if( odom_set_ == false )
	{
		ROS_INFO( "Odometry initialized" );
		odom_set_ = true;
		prev_phi_ = curr_phi_;

	}

	while( curr_phi_ - prev_phi_ < PI )
	{
		curr_phi_ += 2 * PI;
	}

	while( curr_phi_ - prev_phi_ > PI )
	{
		curr_phi_ -= 2 * PI;
	}

	dist_rotated_ += curr_phi_ - prev_phi_;
	prev_phi_ = curr_phi_;
	dist_moved_x_ = curr_x_ - start_x_;
	dist_moved_y_ = curr_y_ - start_y_;

	double distance_moved_x_temp = dist_moved_x_;

	dist_moved_x_ = dist_moved_x_ * cos( -start_phi_ ) - dist_moved_y_ * sin( -start_phi_ );
	dist_moved_y_ = dist_moved_y_ * cos( -start_phi_ ) + distance_moved_x_temp * sin( -start_phi_ );
}


void RobotinoMotionClient::goalCallback( const robotino_motion::MotionGoalConstPtr& msg )
{

	node astar;

	//astar.findpath(mapa, 7, 9, 9, 6);
	astar.findpath();

	robotino_motion::MotionGoal goal;

	ROS_INFO("\nPeenchendo goal");
	goal.move_phi = 0;
	goal.movement_type = 0;
	goal.task_type = 0;
	goal.interruption_condition = 0;
	goal.alignment_device = 0;

	while(!astar.queue_astar.empty())
	{
		ROS_INFO("\nFila nao vazia ");

		goal.move_x = astar.queue_astar.front()*0.25;
		astar.queue_astar.pop();
		goal.move_y = astar.queue_astar.front()*0.25;
		astar.queue_astar.pop();
		queue_.push(goal);
		ROS_INFO("queue_astar: %d, queue_: %d ",astar.queue_astar.size(), queue_.size());
	}
	if(checkServer())
	{
		popGoalCallback(queue_);
	}
}

bool RobotinoMotionClient::moveTo(robotino_motion::MoveTo::Request &req, robotino_motion::MoveTo::Response &res)
{
	robotino_mapping::GetMap service;

	if(req.obstacle_code < 1 || req.obstacle_code > 5)
	{
		service.request.area = req.obstacle_code;
	}
	else
	{
		return false;
	}
	int height = 17;
	int width = 17;
	int virtual_map[height][width];
	if (get_map_cli_.call(service))
	{
		for(int i = 0; i < height; i++)
		{
			for(int j = 0; j < width; j++)
			{
				virtual_map[i][j] = service.response.map[i * height + j];
			}
		}
	}
	else
	{
		return false;
	}

	const int xA=1;
	const int yA=1;
	Indexes goal = convertToIndexes(req.x, req.y);

	const int xB = goal.i;
	const int yB = goal.j;

	node astar;
	astar.findpath((int**) virtual_map,xA,yA,xB,yB);
	/*if ()
	{
		return false;
	}*/

	int real_map[height / 2 + 1][width / 2 + 1];
	for(int i = 0; i < height; i++)
	{
		for(int j = 0; j < width; j++)
		{
			if(i % 2 != 0 && j % 2 != 0)
			{
				real_map[(i - 1) / 2][(j - 1) / 2] = virtual_map[i][j];
			}
		}
	}


	return true;
}

Indexes RobotinoMotionClient::convertToIndexes(Coordinates coordinates)
{
	return convertToIndexes(coordinates.x, coordinates.y);
}

Indexes RobotinoMotionClient::convertToIndexes(float x, float y)
{
	Indexes indexes;
	int delta_x = .5;
	int delta_y = .5;
	indexes.i = ceil(2 * x / delta_x);
	indexes.j = ceil(2 * y / delta_y);
	return indexes;
}

Coordinates RobotinoMotionClient::convertToCoordinates(Indexes indexes)
{
	return convertToCoordinates(indexes.i, indexes.j);
}

Coordinates RobotinoMotionClient::convertToCoordinates(int i, int j)
{
	Coordinates coordinates;
	int delta_x = .5;
	int delta_y = .5;
	coordinates.x = .5 * delta_x * i;
	coordinates.y = .5 * delta_y * j;
	return coordinates;
}

void RobotinoMotionClient::popGoalCallback( const std::queue<robotino_motion::MotionGoal> )
{
	robotino_motion::MotionGoal goal;

		if(!queue_.empty())
		{
			goal = queue_.front();
			queue_.pop();
			std::cout << goal;
			ROS_INFO("%d", queue_.size());
			sendGoal(goal);
		}
		ROS_INFO( "Pushing goal (move_x[m], move_y[m], move_phi[rad], "
					"movement_type, task_type, interruption_condition, alignment_device) = "
					"(%f, %f, %f, %d, %d, %d, %d)",
					goal.move_x, goal.move_y, goal.move_phi, goal.movement_type, goal.task_type,
					goal.interruption_condition, goal.alignment_device);

}

void RobotinoMotionClient::doneCallBack( const actionlib::SimpleClientGoalState& state,
			const robotino_motion::MotionResultConstPtr& result)
{
	ROS_INFO("Finished in state [%s]", state.toString().c_str());
	ROS_INFO("Answer: %i", result->achieved_goal);
	popGoalCallback(queue_);
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
/*
		if( ( ros::Time::now() - start_time ).toSec() > max_time_ )
		{
			ROS_INFO( "Timeout: Aborting Motion" );
			client_.cancelAllGoals();
			break;
		}*/

//Lembrar de compilar o has_arrived ---------------------------------

		robotino_motion::HasArrived has_arrived;
		has_arrived.has_arrived = queue_.empty();

		has_arrived_pub_.publish(has_arrived);

		ros::spinOnce();
		loop_rate.sleep();
	}
}

void RobotinoMotionClient::sendGoal( const robotino_motion::MotionGoal& goal )
{
	client_.sendGoal( goal , boost::bind(&RobotinoMotionClient::doneCallBack, this, _1, _2), Client::SimpleActiveCallback(), Client::SimpleFeedbackCallback());

	ROS_INFO("Goal sent");

}
