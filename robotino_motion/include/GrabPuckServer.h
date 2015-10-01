/**
 *  GrabPuckServer.h
 *
 *  Version: 1.1.0.0
 *  Created on: 01/10/2015
 *  Modified on: ??/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef GRAB_PUCK_SERVER_H_
#define GRAB_PUCK_SERVER_H_

#include <vector>

#include "Server.h"

#include "robotino_motion/GrabPuckAction.h"
#include "GrabPuckStates.h"
#include "Colors.h"

#include "robotino_msgs/DigitalReadings.h"

#include "robotino_vision/FindObjects.h"
	
class GrabPuckServer : public Server
{

public:

	GrabPuckServer(ros::NodeHandle nh, std::string ns);
	~GrabPuckServer();

	bool isActing();	

protected:

	void start();
	void stop();
	void controlLoop();

private:

	ros::Subscriber digital_readings_sub_;
	ros::ServiceClient find_objects_cli_;

	void readParameters();

	/** GrabPuck Action related Variables and Functions */ 	
	actionlib::SimpleActionServer <robotino_motion::GrabPuckAction> server_;
	robotino_motion::GrabPuckGoal goal_;
	robotino_motion::GrabPuckFeedback feedback_;
	robotino_motion::GrabPuckResult result_;

	void executeCallback(const robotino_motion::GrabPuckGoalConstPtr& goal);
	void preemptCallback();
	bool validateNewGoal(const robotino_motion::GrabPuckGoalConstPtr& goal);

	/**  */
	bool is_loaded_;

	void digitalReadingsCallback(const robotino_msgs::DigitalReadings& msg);

	/** Movement related Variables and Functions */
	GrabPuckState state_;

	/** Image Processing Variable and Functions */
	Color color_;
	int nframes_no_puck_; 

};

#endif /* GRAB_PUCK_SERVER_H_ */
