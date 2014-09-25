#include "ros/ros.h"
#include "robotino_arduino/Mag.h"
#include "robotino_arduino/Ultrasound.h"
#include "robotino_arduino/Calibrate.h"
#include "robotino_arduino/GetMagDirection.h"

int val_x = 0;
int val_y = 0;
int val_z = 0;

double a_x = 0, b_x = 0;
double a_y = 0, b_y = 0;
double a_z = 0, b_z = 0;

int min_x = 50, max_x = 0;
int min_y = 50, max_y = 0;
int min_z = 50, max_z = 0;

ros::Time start_time;
float calib_duration;

bool calibrate(robotino_arduino::Calibrate::Request  &req,
         robotino_arduino::Calibrate::Response &res)
{
	start_time = ros::Time::now();
	calib_duration = req.calib_duration;
	min_x = 50;	max_x = 0;
	min_y = 50;	max_y = 0;
	min_z = 50;	max_z = 0;
	ROS_INFO("Starting to Process calibration!!!");
  	return true;
}

bool mag_direction(robotino_arduino::GetMagDirection::Request  &req,
         robotino_arduino::GetMagDirection::Response &res)
{	
	if (a_x == 0 && b_x == 0 && a_y == 0 && b_y == 0 && a_z == 0 && b_z == 0) return false;
	
	double direction;
	double x, y;

	x = (val_x + b_x) / a_x;
	y = (val_y + b_y) / a_y;
	

	direction = atan2 (y,x);
	//ROS_INFO("[%lf]",direction * 180 / 3.14);
	res.mag_angle = (float)direction;
	return true;
	
	}



void magCallback(const robotino_arduino::MagConstPtr& msg)
{
	val_x = msg->x;
	val_y = msg->y;
	val_z = msg->z;

	if ((ros::Time::now() - start_time).toSec() < calib_duration)
	{

		if ( val_x > max_x) max_x = val_x;
		if ( val_x < min_x) min_x = val_x;

		if ( val_y > max_y) max_y = val_y;
		if ( val_y < min_y) min_y = val_y;

		if ( val_z > max_z) max_z = val_z;
		if ( val_z < min_z) min_z = val_z;	

		b_x = -(max_x + min_x) / 2;
		a_x = (max_x - min_x) / 2;
	
		b_y = -(max_y + min_y) / 2;
		a_y = (max_y - min_y) / 2;
	
		b_z = -(max_z + min_z) / 2;
		a_z = (max_z - min_z) / 2;

	}
}

void ultrasoundCallback(const robotino_arduino::UltrasoundConstPtr& msg)
{
  	//ROS_INFO("I heard: [%d, %d]", msg->front, msg->side);
}

int main(int argc, char **argv)
{
  
  	ros::init(argc, argv, "robotino_arduino_node");

  
  	ros::NodeHandle n;

  	ros::Subscriber mag_sub = n.subscribe("mag", 1, magCallback);

  	ros::Subscriber ultrasound_sub = n.subscribe("ultrasound", 1, ultrasoundCallback);

  	ros::ServiceServer calibrate_srv = n.advertiseService("calibrate", calibrate);

	ros::ServiceServer get_mag_direction_srv = n.advertiseService("get_mag_direction", mag_direction);

  	ros::spin();

  	return 0;
}



