/*
 * RobotinoMotionServer.h
 *
 *  Created on: 2014
 *      Author: expertinos.unifei@gmail.com
 */

#ifndef ROBOTINOMOTIONSERVER_H_
#define ROBOTINOMOTIONSERVER_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_datatypes.h>
#include <vector>

#include "robotino_msgs/AnalogReadings.h"
#include "robotino_msgs/DigitalReadings.h"

#include "robotino_motion/MotionAction.h"
#include "robotino_motion/MotionActionGoal.h"
#include "robotino_motion/GetProduct.h"
#include "robotino_motion/Align.h"
#include "robotino_motion/SetAchievedGoal.h"

#include "robotino_vision/FindObjects.h"

#define PI 3.14159
#define sign(a) (((a) < 0) ? -1 : (((a) > 0) ? 1 : 0))

using namespace std;

typedef actionlib::SimpleActionServer<robotino_motion::MotionAction> Server;
typedef enum { IDLE,
		MOVING,
		FINISHED,
		PROCESSING,
		ALIGNING} State;
typedef enum { TRANSLATIONAL,
		ROTATIONAL,
		TRANSLATIONAL_ROTATIONAL,
		TANGENT } MovementType;
typedef enum { ALIGN, 
		MOVE, 
		COUNT, 
		FOLLOW } TaskType;
typedef enum { MOVED_DISTANCE, 
		HIGH_OPTICAL_SIGNAL, 
		LOW_OPTICAL_SIGNAL, 
		HIGH_INDUCTIVE_SIGNAL, 
		LOW_INDUCTIVE_SIGNAL, 
		CAMERA, 
		OBSTACLE,
		BUMPER,
		TIME,
		SECURED_INFRARED_SIGNAL } InterruptionCondition;
typedef enum { NONE, 
		INFRARED, 
		OPTICAL, 
		INDUCTIVE, 
		CAMERAAL, 
		ULTRASONIC,
		COMPASS } AlignmentDevice;
typedef enum { TV, 
		DVD, 
		CELULAR, 
		TABLET, 
		NOTEBOOK } Product;

typedef enum {FRONT, RIGHT, LEFT, BACK} AlignmentMode;

class RobotinoMotionServer
{
public:
	RobotinoMotionServer();
	~RobotinoMotionServer();

private:
	ros::NodeHandle nh_;
	ros::Subscriber odometry_sub_;
	ros::Subscriber scan_sub_;
	ros::Subscriber bumper_sub_;
	ros::Subscriber analog_sub_;
	ros::Subscriber digital_sub_;
	ros::Subscriber distance_sub_;
	ros::Publisher cmd_vel_pub_;
	ros::ServiceServer get_product_srv_;
	ros::ServiceServer align_srv_;
	ros::ServiceClient find_objects_cli_;
	ros::ServiceClient set_achieved_goal_cli_;

	Server server_;

	State state_;
	MovementType movement_type_;
	TaskType task_type_;
	InterruptionCondition interruption_condition_;
	AlignmentDevice alignment_device_;

	robotino_motion::MotionResult result_;
	robotino_motion::MotionFeedback feedback_;

	geometry_msgs::Twist cmd_vel_msg_;
	nav_msgs::Odometry current_odom_msg_;
	nav_msgs::Odometry start_odom_msgs_;
	sensor_msgs::LaserScan laser_scan_msg_;
	std_msgs::Bool bumper_msg_;
	robotino_msgs::AnalogReadings analog_msg_;
	sensor_msgs::PointCloud distance_msg_;
	robotino_msgs::DigitalReadings digital_msg_;

	double curr_x_, curr_y_, curr_phi_, prev_phi_;
	double dist_moved_x_, dist_moved_y_, dist_rotated_;
	double forward_goal_x_, forward_goal_y_, rotation_goal_;
	double start_x_, start_y_, start_phi_;

	double min_linear_vel_;
	double max_linear_vel_;
	double linear_acc_;
	double min_angular_vel_;
	double max_angular_vel_;
	double angular_acc_;
	double percentage_;

	bool odom_set_;
	bool is_loaded_;

	bool ident_obstacle_;
	bool obstacle_;
	int numLaserScan_;
	int IR_[9];

	bool ident_contact_;
	bool contact_;
	bool contact_flag_;

	bool inductive_;
	float inductive_value_;
	std::vector<float> inductive_vector_;

	bool optical_;
	int optical_value_right_;
	int optical_value_left_;
	int optical_value_test_;

	// Image Processing Variable
	int nframes_no_puck_;

	// Back Alignment Variables
	int alignment_mode_;
	int left_index_;
	int right_index_;
	float left_ir_;
	float right_ir_;
	bool lateral_;

	void odomCallback( const nav_msgs::OdometryConstPtr& msg );
	void scanCallback( const sensor_msgs::LaserScan& msg );
	void bumperCallback( const std_msgs::Bool& msg );
	void analogCallback( const robotino_msgs::AnalogReadings& msg );
	void distanceSensorsCallback( const sensor_msgs::PointCloud& msg );
	void digitalCallback( const robotino_msgs::DigitalReadings& msg );
	
	void teleopActivatedCallback( const std_msgs::BoolConstPtr& msg );
	void execute( const robotino_motion::MotionGoalConstPtr& goal );
	void setCmdVel( double vx, double vy, double omega );
	void controlLoop();
	bool acceptNewGoal( const robotino_motion::MotionGoalConstPtr& goal );
	void readParameters( ros::NodeHandle& n);
	void distanceCallback(sensor_msgs::PointCloud& msg);
	bool getProduct(robotino_motion::GetProduct::Request &req, robotino_motion::GetProduct::Response &res);
	void controlImageProcessing();
	bool align(robotino_motion::Align::Request &req, robotino_motion::Align::Response &res);
	void controlAlignment();
	bool setAchievedGoal(bool achieved_goal);

public:
	void spin();
};

#endif /* ROBOTINOMOTIONSERVER_H_ */
