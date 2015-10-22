/**
 *  AlignServer.h
 *
 *  Version: 1.1.0.0
 *  Created on: 01/10/2015
 *  Modified on: ??/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef ALIGN_SERVER_H_
#define ALIGN_SERVER_H_

#include <vector>

#include "Server.h"
#include "AlignStates.h"
#include "AlignAlignmentModes.h"
#include "AlignDistanceModes.h"

#include <sensor_msgs/PointCloud.h>
#include "robotino_motion/AlignAction.h"

#define CLOSE_TOLERANCE 0.20
#define NORMAL_TOLERANCE 0.30
#define FAR_TOLERANCE 0.45
	
class AlignServer : public Server
{

public:

	AlignServer(ros::NodeHandle nh);
	~AlignServer();

	bool isActing();	

protected:

	void start();
	void stop();
	void controlLoop();

private:

	ros::Subscriber distance_sensors_sub_;

	void readParameters();

	/** Align Action related Variables and Functions */ 	
	actionlib::SimpleActionServer<robotino_motion::AlignAction> server_;
	robotino_motion::AlignFeedback feedback_;
	robotino_motion::AlignResult result_;

	void executeCallback(const robotino_motion::AlignGoalConstPtr& goal);
	bool validateNewGoal(const robotino_motion::AlignGoalConstPtr& goal);
	void publishFeedback();

	void distanceSensorsCallback(const sensor_msgs::PointCloud& msg);

	/** Movement related Variables and Functions */
	AlignState state_;
	double percentage_; 

	// Back Alignment Variables
	AlignmentMode alignment_mode_;
	DistanceMode distance_mode_;
	int left_index_, right_index_;
	float left_ir_, right_ir_;
	bool lateral_;

};

#endif /* ALIGN_SERVER_H_ */
