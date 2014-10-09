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
#include "robotino_motion/MotionGoal.h"
#include "robotino_motion/MotionAction.h"
#include "robotino_motion/MotionResult.h"
#include <queue>
#include <actionlib/client/simple_action_client.h>

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

	std::queue<robotino_motion::MotionGoal> queue_;

	float max_time_;

	void goalCallback( const robotino_motion::MotionGoalConstPtr& msg );

	void popGoalCallback( const std::queue<robotino_motion::MotionGoal> );

	void doneCallBack ( const actionlib::SimpleClientGoalState& state,
			const robotino_motion::MotionResultConstPtr& result);

	void activeCb();

	void feedbackCallBack( const robotino_motion::MotionActionFeedbackConstPtr& feedback);



public:
	bool checkServer();
	void spin();
	void setMaxTime( const float& time );
	void sendGoal( const robotino_motion::MotionGoal& goal );


};

#endif /* ROBOTINOMOTIONCLIENT_H_ */
