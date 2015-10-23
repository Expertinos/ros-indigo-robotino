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
	ros::Subscriber laser_scan_sub_;

	void readParameters();

	/** Align Action related Variables and Functions */ 	
	actionlib::SimpleActionServer<robotino_motion::AlignAction> server_;
	robotino_motion::AlignFeedback feedback_;
	robotino_motion::AlignResult result_;

	void executeCallback(const robotino_motion::AlignGoalConstPtr& goal);
	bool validateNewGoal(const robotino_motion::AlignGoalConstPtr& goal);
	void publishFeedback();

	void laserAlignFront();
	void laserAlignRightLeft();

	void distanceSensorsCallback(const sensor_msgs::PointCloud& msg);
	void laserScanCallback(const sensor_msgs::PointCloud& msg);

	/** Movement related Variables and Functions */
	AlignState state_;
	double percentage_; 

	// Back Alignment Variables
	AlignmentMode alignment_mode_;
	DistanceMode distance_mode_;
	int left_index_, right_index_;
	float left_ir_, right_ir_;
	bool lateral_;

	
	//Align using laser
	int index_laser_data_center_;
	int index_laser_data_right_;
	int index_laser_data_left_;
	int angle_increment_;
	double error_laser_data_x_;
	double error_laser_data_y_;

	double laser_data_x_[600];
	double laser_data_y_[600];

	double laser_data_center_x_;
	double laser_data_center_y_;
	double laser_data_center_right_;
	double laser_data_center_left_;
	double laser_data_front_[30];
	double laser_data_left_;
	double laser_data_right_;

};

#endif /* ALIGN_SERVER_H_ */
