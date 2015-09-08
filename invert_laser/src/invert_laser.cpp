#include "invert_laser.h"

Invert_Laser::Invert_Laser(ros::NodeHandle n)
{
	n_ = n;

	scan_sub_ = n.subscribe("scan_uninverted", 10, &Invert_Laser::laserCallback, this);

	scan_pub_ = n.advertise<sensor_msgs::LaserScan>("scan", 50, true);
}

Invert_Laser::~Invert_Laser()
{
	scan_sub_.shutdown();
	scan_pub_.shutdown();
}

void Invert_Laser::laserCallback(const sensor_msgs::LaserScanConstPtr& scan)
{
	laser_data_.header.stamp = scan->header.stamp;
	laser_data_.header.frame_id = scan->header.frame_id;

	laser_data_.angle_min = scan->angle_min;
	laser_data_.angle_max = scan->angle_max;
	laser_data_.angle_increment = scan->angle_increment;

	laser_data_.time_increment = scan->time_increment;
	laser_data_.scan_time = scan->scan_time;

	laser_data_.range_min = scan->range_min;
	laser_data_.range_max = scan->range_max;
	laser_data_.ranges = scan->ranges;

	for (int laser_index = 0 ; laser_index < scan->ranges.size() ; laser_index++ )
	{
		laser_data_.ranges[scan->ranges.size() - laser_index - 1] = scan->ranges[laser_index];
	}

	scan_pub_.publish(laser_data_);

	//std::cout<<"scan_size: "<<scan->ranges.size()<<" | laser_data_size: "<<laser_data_.ranges.size()<<std::endl;

}

