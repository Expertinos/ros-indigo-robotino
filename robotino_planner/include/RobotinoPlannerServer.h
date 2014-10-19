/**
 * RobotinoPlannerServer.h
 *
 *  Created on: 12.10.2014
 *      Author: adrianohrl@unifei.edu.br
 */

#ifndef ROBOTINOPLANNERSERVER_H_
#define ROBOTINOPLANNERSERVER_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <std_msgs/Bool.h>

#include "robotino_planner/PlannerAction.h"
#include "robotino_planner/PlannerActionGoal.h"
#include "robotino_planner/MoveTo.h"
#include "robotino_leds/GoFromTo.h"
#include "robotino_leds/StopTransportation.h"
#include "robotino_leds/TransportProduct.h"


typedef actionlib::SimpleActionServer<robotino_planner::PlannerAction> Server;
typedef enum {MODULE_A, MODULE_B} Module;
typedef enum {NONE, TV, DVD, CELULAR, TABLET, NOTEBOOK} Product;
typedef enum {ORIGIN, SETOR_DE_CONTROLE, EXAMES, CENTRO_CIRURGICO, SETOR_DE_RECUPERACAO, SETOR_DE_SAIDA} Place;

class RobotinoPlannerServer
{
public:
	RobotinoPlannerServer();
	~RobotinoPlannerServer();

	void spin();

private:
	ros::NodeHandle nh_;
	ros::ServiceClient go_srv_;
	ros::ServiceClient move_cli_;
	ros::ServiceClient stop_srv_;
	ros::ServiceClient transport_srv_;
	ros::Subscriber has_arrived_sub_;
	
	Server server_;

	Module module_;

	robotino_planner::PlannerResult result_;
	robotino_planner::PlannerFeedback feedback_;

	// Módulo A variables:
	bool has_arrived_;
	int num_lists_;
	int num_orders_;
	int list_;
	int order_;
	Product** products_;

	// Módulo B variables:
	Place** places_;

	void controlLoop();
	bool acceptNewGoal(const robotino_planner::PlannerGoalConstPtr& goal);
	void execute(const robotino_planner::PlannerGoalConstPtr& goal);
	int getProductCode(Product product);
	int getPlaceCode(Place place);
	void hasArrived(const std_msgs::BoolConstPtr& msg);

};

#endif /* ROBOTINOPLANNERSERVER_H_ */
