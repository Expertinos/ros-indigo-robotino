/*
 * DistanceSensorArrayROS.cpp
 *
 *  Created on: 07.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#include "DistanceSensorArrayROS.h"
#include <cmath>

DistanceSensorArrayROS::DistanceSensorArrayROS()
{
	distances_pub_ = nh_.advertise<sensor_msgs::PointCloud>("distance_sensors", 1, true);
	laser_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("sonar_scan", 1, false);
}

DistanceSensorArrayROS::~DistanceSensorArrayROS()
{
	distances_pub_.shutdown();
	laser_scan_pub_.shutdown();
}

void DistanceSensorArrayROS::setTimeStamp(ros::Time stamp)
{
	stamp_ = stamp;
}

void DistanceSensorArrayROS::distancesChangedEvent(const float* distances, unsigned int size)
{
	// Build the PointCloud msg
	distances_msg_.header.stamp = stamp_;
	distances_msg_.header.frame_id = "base_link";
	distances_msg_.points.resize(size);

	float distances_n[size];
	int convert = 0;

	for(unsigned int i = 0; i < size; ++i)
	{
		convert = (i + 5) % 9;

		distances_n[i] = distances[convert] + 0.2;
	}

	for(unsigned int i = 0; i < size; ++i)
	{
		// 0.698 radians = 40 Degrees
		// 0.2 is the radius of the robot
		distances_msg_.points[i].x = ( distances[i] + 0.2 ) * cos(0.698 * i);
		distances_msg_.points[i].y = ( distances[i] + 0.2 ) * sin(0.698 * i);
		distances_msg_.points[i].z = 0.05; // 5cm above ground
	}

	// Publish the msg
	distances_pub_.publish(distances_msg_);


	//for(unsigned int i = 0; i < size; ++i)
	//{
		// 0.698 radians = 40 Degrees
		// 0.2 is the radius of the robot
		//distances[i]+=0.2;
	//}


	// Build the LaserScan message
	//laser_scan_msg_.header.seq = scan.seq;
	laser_scan_msg_.header.stamp = stamp_;
	laser_scan_msg_.header.frame_id = "laser_link";

	laser_scan_msg_.angle_min = - 160.0 * (M_PI / 180.0);
	laser_scan_msg_.angle_max = 160.0 * (M_PI / 180.0);
	laser_scan_msg_.angle_increment = 40.0 * (M_PI / 180.0);
	laser_scan_msg_.time_increment = 0.1;
	laser_scan_msg_.scan_time = 0.1;
	laser_scan_msg_.range_min = 0.04 + 0.2;
	laser_scan_msg_.range_max = 0.30 + 0.2;

	laser_scan_msg_.ranges.resize( size );
	laser_scan_msg_.intensities.resize( size);

	//ROS_INFO(" num intensities: %d num ranges: %d", numIntensities, numRanges );
	//if( ranges != NULL )
	//{
		memcpy( laser_scan_msg_.ranges.data(), distances_n, size * sizeof(float) );
	//}

	//if( intensities != NULL )
	//{
		memcpy( laser_scan_msg_.intensities.data(), distances_n, size * sizeof(float) );
	//}

	// Publish the message
	//if( numRanges > 0 || numIntensities > 0)
		laser_scan_pub_.publish(laser_scan_msg_);


}
