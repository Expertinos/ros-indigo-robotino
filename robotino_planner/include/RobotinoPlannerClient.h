/*
 * RobotinoPlannerClient.h
 *
 *  Created on: 12.10.2014
 *      Author: adrianohrl@unifei.edu.br
 */

#ifndef ROBOTINOPLANNERCLIENT_H_
#define ROBOTINOPLANNERCLIENT_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>
#include "robotino_planner/PlannerGoal.h"
#include "robotino_planner/PlannerAction.h"
#include "robotino_planner/Abort.h"
#include "robotino_planner/StartModule.h"

#include "robotino_motion/MotionGoal.h"
#include "robotino_motion/GetProduct.h"
#include "robotino_motion/Align.h"
#include "robotino_motion/AchievedGoal.h"

#include "robotino_leds/GoFromTo.h"
#include "robotino_leds/SinalizeEnd.h"
#include "robotino_leds/StopTransportation.h"

#include "robotino_vision/GetProductsList.h"

typedef enum {NONE, RED, YELLOW, BLUE, GREEN} Color;
typedef enum {FRONT, RIGHT, LEFT, BACK} AlignmentMode;

typedef actionlib::SimpleActionClient<robotino_planner::PlannerAction> Client;

using namespace std;

class RobotinoPlannerClient
{
public:
	RobotinoPlannerClient();
	~RobotinoPlannerClient();

	bool checkServer();
	void spin();
	void sendGoal(const robotino_planner::PlannerGoal& goal);

private:
	ros::NodeHandle nh_;

	ros::ServiceServer abort_srv_;
	ros::ServiceServer start_srv_;

	ros::Publisher goal_pub_;
	
	ros::ServiceClient hold_patient_cli_;
	ros::ServiceClient align_cli_;
	ros::ServiceClient go_to_cli_;
	ros::ServiceClient sinalize_end_cli_;
	ros::ServiceClient stop_transport_cli_;
	ros::ServiceClient achieved_goal_cli_;
	ros::ServiceClient get_products_list_cli_;

	Client client_;

	robotino_planner::PlannerGoal goal_;

	bool abort(robotino_planner::Abort::Request &req, robotino_planner::Abort::Response &res);
	bool start(robotino_planner::StartModule::Request &req, robotino_planner::StartModule::Response &res);

	void publishGoal(float move_x, float move_y, float move_phi, int movement_type);
	bool hasBeenAchieved();
	bool holdPatient();
	bool setLeds(Color color);
	bool align(AlignmentMode mode);
	Color getOrder();
	bool stopTransportation();
	void movingGREEN();
	void movingRED();
	void movingYELLOW();
	void movingBLUE();

};

#endif /* ROBOTINOPLANNERCLIENT_H_ */
