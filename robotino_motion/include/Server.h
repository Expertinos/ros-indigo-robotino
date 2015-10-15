/**
 *  Server.h
 *
 *  Version: 1.0.0.0
 *  Created on: 01/10/2015
 *  Modified on: ??/10/2014
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef SERVER_H_
#define SERVER_H_

#include <vector>
#include <string>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <opencv2/highgui/highgui.hpp> //apenas para a sintonia através de trackbars

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_datatypes.h>

#define MIN_LINEAR_VELOCITY -0.20
#define MIN_ANGULAR_VELOCITY -1.20
#define MAX_LINEAR_VELOCITY 0.20
#define MAX_ANGULAR_VELOCITY 1.20
#define PI 3.14159
#define sign(a) (((a) < 0) ? -1 : (((a) > 0) ? 1 : 0))


class Server {

public:

	Server(ros::NodeHandle nh, std::string name);
	~Server();

	void spin();
	std::string getName() const;
	void executeEmergencyStop();	

protected:

	ros::NodeHandle nh_;
	std::string name_;

	void setVelocity(double vel_x, double vel_y, double vel_phi);
	void publishVelocity();
	void resetOdometry();
	double getOdometry_X();
	double getOdometry_Y();
	double getOdometry_PHI();

	virtual void start() = 0;
	virtual void stop() = 0;
	virtual void controlLoop() = 0;

private:

	geometry_msgs::Twist cmd_vel_msg_;
	ros::Publisher cmd_vel_pub_;
	ros::Subscriber odom_sub_;

	bool odom_setted_;
	
	double start_x_, start_y_, start_phi_;
	double curr_x_, curr_y_, curr_phi_, prev_phi_;
	double disp_x_, disp_y_, disp_phi_;

	void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
	void readParameters();

};

#endif /* SERVER_H_ */
