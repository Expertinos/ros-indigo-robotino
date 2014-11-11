/*
 * RobotinoMotionClient.h
 *
 *  Created on: 2014
 *      Author: expertinos.unifei@gmail.com
 */

#ifndef ROBOTINOMOTIONCLIENT_H_
#define ROBOTINOMOTIONCLIENT_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <queue>
#include <iostream>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include "robotino_motion/MotionGoal.h"
#include "robotino_motion/MotionAction.h"
#include "robotino_motion/MotionResult.h"
#include "robotino_motion/HasArrived.h"
#include "robotino_motion/MoveTo.h"
#include "robotino_motion/AchievedGoal.h"
#include "robotino_mapping/GetMap.h"
#include "robotino_motion/SetAchievedGoal.h"
#include "RobotinoMotionServer.h"
#include "astar.h"

typedef actionlib::SimpleActionClient<robotino_motion::MotionAction> Client;

class RobotinoMotionClient
{
public:
	RobotinoMotionClient();
	~RobotinoMotionClient();

private:

	ros::NodeHandle nh_;

	ros::Subscriber goal_sub_;

	ros::Publisher has_arrived_pub_;

	ros::ServiceServer move_srv_;

	ros::ServiceServer achieved_goal_srv_;

	ros::ServiceServer set_achieved_goal_srv_;

	ros::ServiceClient get_map_cli_;

	ros::Subscriber odometry_sub_cli_;

	Client client_;

	robotino_motion::MotionGoal goal_;

	std::queue<robotino_motion::MotionGoal> queue_;

	float max_time_;

	void goalCallback( const robotino_motion::MotionGoalConstPtr& msg );

	void popGoalCallback( const std::queue<robotino_motion::MotionGoal> );

	void doneCallBack ( const actionlib::SimpleClientGoalState& state,
			const robotino_motion::MotionResultConstPtr& result);

	bool moveTo(robotino_motion::MoveTo::Request &req, robotino_motion::MoveTo::Response &res);

	bool setAchievedGoal(robotino_motion::SetAchievedGoal::Request &req, robotino_motion::SetAchievedGoal::Response &res);

	bool achievedGoal(robotino_motion::AchievedGoal::Request &req, robotino_motion::AchievedGoal::Response &res);

	void odomCallback( const nav_msgs::OdometryConstPtr& msg );

	robotino_motion::MotionGoal buildGoal(float move_x, float move_y, float move_phi, int movement_type, int task_type, int interruption_condition, int alignment_device);
	
	double curr_x_, curr_y_, curr_phi_, prev_phi_;
	double dist_moved_x_, dist_moved_y_, dist_rotated_;
	double forward_goal_x_, forward_goal_y_, rotation_goal_;
	double start_x_, start_y_, start_phi_;
	bool odom_set_;
	bool achieved_goal_;

public:
	bool checkServer();
	void spin();
	void setMaxTime( const float& time );
	void sendGoal( const robotino_motion::MotionGoal& goal );

};

#endif /* ROBOTINOMOTIONCLIENT_H_ */
