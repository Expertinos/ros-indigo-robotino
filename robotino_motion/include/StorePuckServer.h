/**
 *  StorePuckServer.h
 *
 *  Version: 1.1.0.0
 *  Created on: 22/10/2015
 *  Modified on: ??/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef STORE_PUCK_SERVER_H_
#define STORE_PUCK_SERVER_H_

#include <vector>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "Server.h"
#include "StorePuckStates.h"
#include "StoreModes.h"
#include "AlignStates.h"
#include "AlignAlignmentModes.h"
#include "AlignDistanceModes.h"

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include "robotino_motion/StorePuckAction.h"
#include "robotino_motion/AlignAction.h"
#include "robotino_msgs/DigitalReadings.h"
#include "robotino_vision/FindInsulatingTapeAreas.h"

#define STORING_DEADLINE 2 // seconds
	
class StorePuckServer : public Server
{

public:

	StorePuckServer(ros::NodeHandle nh);
	~StorePuckServer();

	bool isActing();	

protected:

	void start();
	void stop();
	void controlLoop();

private:

	ros::Subscriber distance_sensors_sub_;
	ros::Subscriber digital_readings_sub_;
	ros::Subscriber laser_scan_sub_;
	ros::ServiceClient find_areas_cli_;

	void readParameters();

	/** StorePuck Action related Variables and Functions */ 	
	actionlib::SimpleActionServer<robotino_motion::StorePuckAction> server_;
	robotino_motion::StorePuckFeedback feedback_;
	robotino_motion::StorePuckResult result_;

	actionlib::SimpleActionClient<robotino_motion::AlignAction> align_client_;

	void executeCallback(const robotino_motion::StorePuckGoalConstPtr& goal);
	bool validateNewGoal(const robotino_motion::StorePuckGoalConstPtr& goal);
	void publishFeedback();

	/**  */
	StoreMode mode_;
	bool loaded_;

	void distanceSensorsCallback(const sensor_msgs::PointCloud& msg);
	void digitalReadingsCallback(const robotino_msgs::DigitalReadings& msg);
	void laserScanCallback(const sensor_msgs::LaserScan& msg);

	/** Movement related Variables and Functions */
	StorePuckState state_;
	double percentage_;
	ros::Time storing_start_;
	double delta_x_;

	// Back Alignment Variables
	int left_index_, right_index_;
	float left_ir_, right_ir_;
	bool lateral_;

	double laser_front_;
	double laser_right_;
	double laser_left_;
	double phi_fut_;

};

#endif /* STORE_PUCK_SERVER_H_ */
