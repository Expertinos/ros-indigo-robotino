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
#include "robotino_motion/MoveTo.h"
#include "astar.h"
#include "robotino_mapping/GetMap.h"
#include "robotino_motion/HasArrived.h"
#include "RobotinoMotionServer.h"
#include <iostream>
#include <vector>
#include <queue>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

using namespace std;


typedef actionlib::SimpleActionClient<robotino_motion::MotionAction> Client;
/*
typedef enum {HOME, TV, DVD, CELULAR, TABLET, NOTEBOOK, ORDER_1, ORDER_2} AreaA;
typedef enum {ORIGIN, ELEVATOR, SETOR_DE_CONTROLE, EXAMES, CENTRO_CIRURGICO, SETOR_DE_RECUPERACAO, SETOR_DE_SAIDA} AreaB;
typedef enum {MODULE_A, MODULE_B} Module;
*/

struct Coordinates
{
  double x;
  double y;
};

struct Indexes
{
  int i;
  int j;
};

class RobotinoMotionClient
{
public:
	RobotinoMotionClient();
	~RobotinoMotionClient();

private:

	ros::NodeHandle nh_;

	ros::Subscriber goal_sub_;

	ros::ServiceServer move_srv_;

	ros::Publisher has_arrived_pub_;

	ros::ServiceClient get_map_cli_;

	ros::Subscriber odometry_sub_cli_;

	Client client_;

	robotino_motion::MotionGoal goal_;

	std::queue<robotino_motion::MotionGoal> queue_;

	void goalCallback( const robotino_motion::MotionGoalConstPtr& msg );

	void popGoalCallback( const std::queue<robotino_motion::MotionGoal> );

	void doneCallBack ( const actionlib::SimpleClientGoalState& state,
			const robotino_motion::MotionResultConstPtr& result);

	Indexes convertToIndexes(Coordinates coordinates);

	Indexes convertToIndexes(float x, float y);

	Coordinates convertToCoordinates(Indexes indexes);

	Coordinates convertToCoordinates(int i, int j);

	double curr_x_, curr_y_, curr_phi_, prev_phi_;
	double dist_moved_x_, dist_moved_y_, dist_rotated_;
	double forward_goal_x_, forward_goal_y_, rotation_goal_;
	double start_x_, start_y_, start_phi_;
	bool odom_set_;

public:
	bool checkServer();
	void spin();
	void sendGoal( const robotino_motion::MotionGoal& goal );
	bool moveTo(robotino_motion::MoveTo::Request &req, robotino_motion::MoveTo::Response &res);
	void odomCallback( const nav_msgs::OdometryConstPtr& msg );

};

#endif /* ROBOTINOMOTIONCLIENT_H_ */
