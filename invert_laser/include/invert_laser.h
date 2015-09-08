#ifndef INVERT_LASER_H
#define INVERT_LASER_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include <stdio.h>

class Invert_Laser
{
	public:
		Invert_Laser(ros::NodeHandle n);
		virtual ~Invert_Laser();
	public:
		void laserCallback (const sensor_msgs::LaserScanConstPtr& scan);

	private:
		ros::NodeHandle n_;
		ros::Subscriber scan_sub_;
		ros::Publisher scan_pub_;

		sensor_msgs::LaserScan laser_data_;

};

#endif
