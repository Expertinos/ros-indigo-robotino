/*
 * main.cpp
 *
 * Created on: 13/10/2014
 * Author: adrianohrl@unifei.edu.br
 */

#include <ros/ros.h>
#include "RobotinoMapping.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "robotino_leds_node");

	RobotinoMapping rm;
	rm.spin();
	return 0;
}
