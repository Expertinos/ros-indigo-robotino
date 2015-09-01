/*
 * RobotinoLocalMoveServer.h
 *
 *  Created on: 13.12.2011
 *      Author: adrianohrl@unifei.edu.br
 */

#ifndef ROBOTINOLOCALMOVESERVER_H_
#define ROBOTINOLOCALMOVESERVER_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>

#include "robotino_local_move/LocalMoveAction.h"
#include "robotino_local_move/LocalMoveActionGoal.h"

#define PI 3.14159
#define VEL_LIN 0.2 // 
#define VEL_ANG 0.5  // 
#define sign(a) (((a) < 0) ? -1 : (((a) > 0) ? 1 : 0))

typedef actionlib::SimpleActionServer<robotino_local_move::LocalMoveAction> Server;
typedef enum {IDLE, TRANSLATIONAL_MOVEMENT, ROTATIONAL_MOVEMENT, TRANSLATIONAL_ROTATIONAL_MOVEMENT, TANGENT_MOVEMENT, FINISHED} State;

class RobotinoLocalMoveServer
{
public:
	RobotinoLocalMoveServer();
	~RobotinoLocalMoveServer();

private:
	ros::NodeHandle nh_;

	ros::Subscriber odometry_sub_;
	ros::Subscriber scan_sub_;

	ros::Publisher cmd_vel_pub_;

	Server server_;

	State state_;

	robotino_local_move::LocalMoveResult result_;
	robotino_local_move::LocalMoveFeedback feedback_;

	geometry_msgs::Twist cmd_vel_msg_;
	nav_msgs::Odometry current_odom_msg_;
	nav_msgs::Odometry start_odom_msgs_;

	double curr_x_, curr_y_, curr_phi_, prev_phi_;
	double dist_moved_x_, dist_moved_y_, dist_rotated_;
	double forward_goal_x_, forward_goal_y_, rotation_goal_;
	double start_x_, start_y_, start_phi_;

	std::vector<geometry_msgs::Point32> forward_vel_vector_;
	std::vector<geometry_msgs::Point32> rotation_vel_vector_;

	bool odom_set_;
	bool ident_obstacle_;
	bool obstacle_;

	void odomCallback(const nav_msgs::OdometryConstPtr& msg);
	void scanCallback(const sensor_msgs::LaserScan& msg);
	void teleopActivatedCallback(const std_msgs::BoolConstPtr& msg);
	void execute(const robotino_local_move::LocalMoveGoalConstPtr& goal);
	void setCmdVel(double vel_x, double vel_y, double vel_phi);
	void controlLoop();
	bool acceptNewGoal(const robotino_local_move::LocalMoveGoalConstPtr& goal);
	void readParameters(ros::NodeHandle& n);

public:
	void spin();
};

#endif /* ROBOTINOLOCALMOVESERVER_H_ */
