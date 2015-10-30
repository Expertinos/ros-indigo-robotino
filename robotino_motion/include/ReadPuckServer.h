/**
 *  ReadPuckServer.h
 *
 *  Version: 1.1.0.0
 *  Created on: 01/10/2015
 *  Modified on: ??/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef READ_PUCK_SERVER_H_
#define READ_PUCK_SERVER_H_

#include <vector>

#include <actionlib/client/simple_action_client.h>

#include "Server.h"
#include "ReadPuckStates.h"
#include "Colors.h"

#include "robotino_motion/ReadPuckAction.h"
#include "robotino_motion/AlignAction.h"
#include "robotino_vision/FindObjects.h"

#define READING_DEADLINE 2.5 //seconds
	
struct Puck {
	Color color;
	int number_of_markers;
	int counter;
};

class ReadPuckServer : public Server
{

public:

	ReadPuckServer(ros::NodeHandle nh);
	~ReadPuckServer();

	bool isActing();	

protected:

	void start();
	void stop();
	void controlLoop();

private:

	ros::ServiceClient find_objects_cli_;

	void readParameters();

	/** ReadPuck Action related Variables and Functions */ 	
	actionlib::SimpleActionClient<robotino_motion::AlignAction> align_client_;
	actionlib::SimpleActionServer<robotino_motion::ReadPuckAction> server_;
	robotino_motion::ReadPuckFeedback feedback_;
	robotino_motion::ReadPuckResult result_;

	void executeCallback(const robotino_motion::ReadPuckGoalConstPtr& goal);
	bool validateNewGoal(const robotino_motion::ReadPuckGoalConstPtr& goal);
	void publishFeedback();

	void updatePucks(Puck puck);

	/** Movement related Variables and Functions */
	ReadPuckState state_;
	double percentage_;
	ros::Time reading_start_;
	double delta_x_;

	/** Image Processing Variable and Functions */
	std::vector<Color> valid_colors_;
	bool verify_markers_;
	std::vector<int> valid_number_of_markers_;
	std::vector<Puck> pucks_;
};

#endif /* READ_PUCK_SERVER_H_ */
