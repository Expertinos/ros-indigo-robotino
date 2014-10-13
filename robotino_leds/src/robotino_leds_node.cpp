/*
 * main.cpp
 *
 * Created on: 07/10/2014
 * Author: adrianohrl@unifei.edu.br
 */


#include "RobotinoLeds.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "robotino_leds_node");

	RobotinoLeds rl;
	rl.spin();
	return 0;
}
