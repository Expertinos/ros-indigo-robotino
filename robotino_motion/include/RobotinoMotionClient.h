/*
 * RobotinoMotionClient.h
 *
 *  Created on: 13.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#ifndef ROBOTINOMOTIONCLIENT_H_
#define ROBOTINOMOTIONCLIENT_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "robotino_motion/MotionGoal.h"
#include "robotino_motion/MotionAction.h"

typedef actionlib::SimpleActionClient<robotino_motion::MotionAction> Client;

class RobotinoMotionClient
{
public:
	RobotinoMotionClient();
	~RobotinoMotionClient();

private:
	ros::NodeHandle nh_;

	ros::Subscriber goal_sub_;

	Client client_;

	robotino_motion::MotionGoal goal_;

	float max_time_;

	void goalCallback( const robotino_motion::MotionGoalConstPtr& msg );

public:
	bool checkServer();
	void spin();
	void setMaxTime( const float& time );
	void sendGoal( const robotino_motion::MotionGoal& goal );

};

#endif /* ROBOTINOMOTIONCLIENT_H_ */
