/**
 *  Server.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 01/10/2015
 *  Modified on: ??/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "Server.h"

/**
 *
 */
Server::Server(ros::NodeHandle nh, std::string name, std::string ns) 
{
	nh_ = nh;
	name_ = name;
	odom_setted_ = false;
	readParameters();
	cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/" + ns + "cmd_vel", 1);
	odom_sub_ = nh_.subscribe("/" + ns + "odom", 1, &Server::odometryCallback, this);
}

/**
 *
 */
Server::~Server() 
{
	cmd_vel_pub_.shutdown();
	odom_sub_.shutdown();
}

/**
 *
 */
void Server::spin() 
{
	ROS_INFO("%s Server up and running!!!", name_.c_str());
	ros::Rate loop_rate(10.0);
	while (nh_.ok()) {
		//controlLoop();
		ros::spinOnce();
		loop_rate.sleep();
	}
}

/**
 *
 */
std::string Server::getName() const 
{
	return name_;
}

/**
 *
 */
void Server::executeEmergencyStop()
{
	setVelocity(0, 0, 0);
	publishVelocity();
	stop();
}	

/**
 *
 */
void Server::setVelocity(double vel_x, double vel_y, double vel_phi) 
{
	if (fabs(vel_x) > max_linear_vel_) 
	{
		vel_x = sign(vel_x) * max_linear_vel_;
	}
	if (fabs(vel_y) > max_linear_vel_) 
	{
		vel_y = sign(vel_y) * max_linear_vel_;
	}
	if (fabs(vel_phi) > max_angular_vel_) 
	{
		vel_phi = sign(vel_phi) * max_angular_vel_;
	}
	cmd_vel_msg_.linear.x = vel_x;
	cmd_vel_msg_.linear.y = vel_y;
	cmd_vel_msg_.angular.z = vel_phi;
}

/**
 *
 */
void Server::resetOdometry() 
{
	start_x_ = curr_x_;
	start_y_ = curr_y_;
	start_phi_ = curr_phi_;
	disp_x_ = 0.0;
	disp_y_ = 0.0;
	disp_phi_ = 0.0;
}

/**
 *
 */
double Server::getOdometry_X() 
{
	return disp_x_;

}
/**
 *
 */
double Server::getOdometry_Y() 
{
	return disp_y_;

}
/**
 *
 */
double Server::getOdometry_PHI() 
{
	return disp_phi_;

}

/**
 *
 */
void Server::publishVelocity() 
{
	cmd_vel_pub_.publish(cmd_vel_msg_);
}

/**
 *
 */
void Server::odometryCallback(const nav_msgs::OdometryConstPtr& msg) 
{
	curr_x_ = msg->pose.pose.position.x;
	curr_y_ = msg->pose.pose.position.y;
	curr_phi_ = tf::getYaw(msg->pose.pose.orientation);
	if (!odom_setted_) {
		ROS_INFO("Odometry initialized!!!");
		odom_setted_ = true;
		prev_phi_ = curr_phi_;
		resetOdometry();
		start();
	}
	disp_x_ = (curr_x_ - start_x_) * cos(-start_phi_) - (curr_y_ - start_y_) * sin(-start_phi_);
	disp_y_ = (curr_y_ - start_y_) * cos(-start_phi_) + (curr_x_ - start_x_) * sin(-start_phi_);
	while (curr_phi_ - prev_phi_ < PI) {
		curr_phi_ += 2 * PI;
	}
	while (curr_phi_ - prev_phi_ > PI) {
		curr_phi_ -= 2 * PI;
	}
	disp_phi_ += curr_phi_ - prev_phi_;
	prev_phi_ = curr_phi_;
}

/**
 *
 */
void Server::readParameters() 
{
	max_linear_vel_ = 500;
	max_angular_vel_ = 1000;	
}
