/*
 * main.cpp
 *
 * Created on: 17/07/2014
 * Author: adrianohrl@unifei.edu.br
 */


#include "RobotinoVision.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "robotino_vision_node");

	RobotinoVision rv;
	rv.spin();
	return 0;
}
