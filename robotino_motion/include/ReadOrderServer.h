/**
 *  ReadOrderServer.h
 *
 *  Version: 1.1.0.0
 *  Created on: 01/10/2015
 *  Modified on: ??/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef READ_ORDER_SERVER_H_
#define READ_ORDER_SERVER_H_

#include <vector>

#include "Server.h"
#include "ReadOrderStates.h"
#include "Colors.h"

#include <actionlib/client/simple_action_client.h>

#include "robotino_motion/ReadOrderAction.h"
#include "robotino_motion/AlignAction.h"
#include "robotino_msgs/DigitalReadings.h"
#include "robotino_vision/GetObjectsList.h"
#include "robotino_vision/FindObjects.h"

#define READING_DEADLINE 2 // seconds
	
class ReadOrderServer : public Server
{

public:

	ReadOrderServer(ros::NodeHandle nh);
	~ReadOrderServer();

	bool isActing();	

protected:

	void start();
	void stop();
	void controlLoop();

private:

	ros::ServiceClient get_list_cli_;

	void readParameters();

	/** ReadOrder Action related Variables and Functions */ 	
	actionlib::SimpleActionClient<robotino_motion::AlignAction> align_client_;
	actionlib::SimpleActionServer<robotino_motion::ReadOrderAction> server_;
	robotino_motion::ReadOrderFeedback feedback_;
	robotino_motion::ReadOrderResult result_;

	bool allDifferentObjects(std::vector<Color> list);
	bool containList(std::vector<Color> list);
	int getListIndex(std::vector<Color> list);
	void executeCallback(const robotino_motion::ReadOrderGoalConstPtr& goal);
	bool validateNewGoal(const robotino_motion::ReadOrderGoalConstPtr& goal);
	void publishFeedback();

	/** Movement related Variables and Functions */
	ReadOrderState state_;
	double percentage_;
	ros::Time reading_start_;
	double delta_x_;

	/** Image Processing Variable and Functions */
	std::vector<Color> valid_colors_;
	int valid_number_of_objects_;
	int final_list_index_;
	std::vector<std::vector<Color> > lists_;
	std::vector<int> counters_;
};

#endif /* READ_ORDER_SERVER_H_ */
