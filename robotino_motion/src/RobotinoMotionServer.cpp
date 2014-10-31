/*
 * RobotinoMotionServer.cpp
 *
 *  Created on: 2014
 *      Author: expertinos.unifei@gmail.com
 */

#include "RobotinoMotionServer.h"

RobotinoMotionServer::RobotinoMotionServer():
	//nh_("~"),
	server_ ( nh_, "motion",
		boost::bind( &RobotinoMotionServer::execute, this, _1 ), false ),
	curr_x_( 0.0 ),
	curr_y_( 0.0 ),
	curr_phi_( 0.0 ),
	prev_phi_( 0.0 ),
	dist_moved_x_( 0.0 ),
	dist_moved_y_( 0.0 ),
	dist_rotated_( 0.0 ),
	forward_goal_x_( 0.0 ),
	forward_goal_y_( 0.0 ),
	rotation_goal_( 0.0 ),
	start_x_( 0.0 ),
	start_y_( 0.0 ),
	start_phi_( 0.0 ),
	odom_set_(false )
{
	odometry_sub_ = nh_.subscribe("odom", 1, &RobotinoMotionServer::odomCallback, this);
	scan_sub_ = nh_.subscribe("scan", 10, &RobotinoMotionServer::scanCallback, this);
	bumper_sub_ = nh_.subscribe("bumper", 1, &RobotinoMotionServer::bumperCallback, this);
	analog_sub_ = nh_.subscribe("analog_readings", 1, &RobotinoMotionServer::analogCallback, this);
	distance_sub_ = nh_.subscribe("distance_sensors", 1, &RobotinoMotionServer::distanceSensorsCallback, this);
	digital_sub_ = nh_.subscribe("digital_readings", 1, &RobotinoMotionServer::digitalCallback, this);

	cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1 );	

	get_product_srv_ = nh_.advertiseService("get_product", &RobotinoMotionServer::getProduct, this);
	align_srv_ = nh_.advertiseService("align", &RobotinoMotionServer::align, this);

	find_objects_cli_ = nh_.serviceClient<robotino_vision::FindObjects>("find_objects");
	set_achieved_goal_cli_ = nh_.serviceClient<robotino_motion::SetAchievedGoal>("set_achieved_goal");

	state_ = IDLE;
	movement_type_ = TRANSLATIONAL;
	task_type_ = ALIGN;
	interruption_condition_ = MOVED_DISTANCE;
	alignment_device_ = NONE;

	readParameters(nh_);

	is_loaded_ = false;

	ident_obstacle_ = true;
	obstacle_ = false;
	numLaserScan_ = 0;

	ident_contact_ = true;
	contact_ = false;
	contact_flag_ = false;

	inductive_ = false;
	inductive_vector_.push_back(0.0);

	optical_ = false;
	
	left_index_ = 0;
	right_index_ = 0;
	lateral_ = false;
}

RobotinoMotionServer::~RobotinoMotionServer()
{
	odometry_sub_.shutdown();
	cmd_vel_pub_.shutdown();
	server_.shutdown();
	scan_sub_.shutdown();
	bumper_sub_.shutdown();
	set_achieved_goal_cli_.shutdown();
}
	
void RobotinoMotionServer::distanceSensorsCallback(const sensor_msgs::PointCloud& msg)
{
	if (!lateral_)
	{
		left_ir_ = msg.points[left_index_].x; 
		right_ir_ = msg.points[right_index_].x; 
	}
	else
	{
		left_ir_ = msg.points[left_index_].y; 
		right_ir_ = msg.points[right_index_].y;
	}
}

void RobotinoMotionServer::odomCallback( const nav_msgs::OdometryConstPtr& msg )
{
	curr_x_ = msg->pose.pose.position.x;
	curr_y_ = msg->pose.pose.position.y;
	curr_phi_ = tf::getYaw( msg->pose.pose.orientation );

	if( odom_set_ == false )
	{
		ROS_INFO( "Odometry initialized" );
		odom_set_ = true;
		prev_phi_ = curr_phi_;

		server_.start();
	}

	while( curr_phi_ - prev_phi_ < PI )
	{
		curr_phi_ += 2 * PI;
	}

	while( curr_phi_ - prev_phi_ > PI )
	{
		curr_phi_ -= 2 * PI;
	}

	dist_rotated_ += curr_phi_ - prev_phi_;
	prev_phi_ = curr_phi_;
	dist_moved_x_ = curr_x_ - start_x_;
	dist_moved_y_ = curr_y_ - start_y_;

	double distance_moved_x_temp = dist_moved_x_;

	dist_moved_x_ = dist_moved_x_ * cos( -start_phi_ ) - dist_moved_y_ * sin( -start_phi_ );
	dist_moved_y_ = dist_moved_y_ * cos( -start_phi_ ) + distance_moved_x_temp * sin( -start_phi_ );
}

void RobotinoMotionServer::scanCallback(const sensor_msgs::LaserScan& msg )
{
	obstacle_ = false;
	if(ident_obstacle_ == true)
	{
		for(int j=0;j<9;j++)
		{
			IR_[j] = 0;
		}
		if(msg.ranges[0] < 0.25)
		{
			obstacle_ = true;
			IR_[0] = 1;
		}
		for(numLaserScan_ = 1; numLaserScan_ < 9; numLaserScan_++)
		{
			if(msg.ranges[numLaserScan_] < 0.59)
			{
				obstacle_ = true;
				IR_[numLaserScan_] = 1;
			}
		}
	}
}

void RobotinoMotionServer::bumperCallback(const std_msgs::Bool& msg )
{
	contact_ = false;
	if(msg.data == true || contact_flag_ == true)
	{
		contact_ = true;
		//ROS_INFO("Contact = true \n");
		//contact_flag_ = true;
	}

}

void RobotinoMotionServer::analogCallback(const robotino_msgs::AnalogReadings& msg )
{
	inductive_ = false;
	inductive_value_ = msg.values.at(0);

	if(inductive_value_ < 5)
	{
		inductive_ = true;
		//ROS_INFO("Inductive = true \n");
	}
}

void RobotinoMotionServer::digitalCallback(const robotino_msgs::DigitalReadings& msg )
{
	
	optical_ = false;
	//ROS_INFO("Optical = false \n");
	optical_value_right_ = msg.values.at(4);
	optical_value_left_ = msg.values.at(2);
	optical_value_test_ = msg.values.at(0);
	is_loaded_ = !msg.values.at(4);

	//ROS_INFO("Optical Right: %i || Optical Left: %i", msg.values.at(4),msg.values.at(2));

}

void RobotinoMotionServer::execute( const robotino_motion::MotionGoalConstPtr& goal )
{
	ros::Rate loop_rate( 10 );
	if( !acceptNewGoal( goal ) )
	{
		ROS_WARN( "Goal not accepted" );
		return;
	}

	while( nh_.ok() )
	{
		if( server_.isPreemptRequested() )
		{
			if( server_.isNewGoalAvailable() )
			{
				if( acceptNewGoal( server_.acceptNewGoal() ) == false )
				{
					return;
				}
			}
			else
			{
				ROS_INFO( "Cancel request" );
				server_.setPreempted();
				state_ = IDLE;
				setCmdVel( 0, 0, 0 );

				return;
			}
		}

/*		if ((obstacle_ == true)||(contact_ == true)||(inductive_ == true))
		{
			setCmdVel(0, 0, 0);
			cmd_vel_pub_.publish(cmd_vel_msg_);

			if(obstacle_ == true)
			{
				for(int j=0;j<9;j++)
				{
					ROS_INFO("Obstacle = true -- Sensor IR = %d, ", IR_[j]);
				}
				ROS_INFO("\n");
			}
		}
*/
		else
		{
			controlLoop();
		}

		if( state_ == FINISHED )
		{
			setCmdVel( 0, 0, 0 );
			cmd_vel_pub_.publish( cmd_vel_msg_ );

			result_.achieved_goal = true;
			server_.setSucceeded( result_ );
			state_ = IDLE;
			/*ROS_INFO( "Motion execution complete: (x[m], y[m], phi[deg]): (%f, %f, %f)",
					dist_moved_x_, dist_moved_y_, ( dist_rotated_ * 180 ) / PI );*/

			return;
		}

		if( state_ != IDLE )
		{
			cmd_vel_pub_.publish( cmd_vel_msg_ );
			server_.publishFeedback( feedback_ );
		}

		ros::spinOnce();
		loop_rate.sleep();

	}

	setCmdVel( 0, 0, 0 );
	cmd_vel_pub_.publish( cmd_vel_msg_ );

	server_.setAborted(robotino_motion::MotionResult(),
			"Aborting on the goal because the node has been killed");
}

void RobotinoMotionServer::setCmdVel( double vx, double vy, double omega )
{
	cmd_vel_msg_.linear.x = vx;
	cmd_vel_msg_.linear.y = vy;
	cmd_vel_msg_.angular.z = omega;
}

void RobotinoMotionServer::spin()
{
	ros::Rate loop_rate ( 5 );

	ROS_INFO("Robotino Motion Server up and running!!!");
	
	while( nh_.ok() )
	{
		if (state_ == PROCESSING)
		{
			controlImageProcessing();
		}
		else if (state_ == ALIGNING)
		{
			controlAlignment();
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
}

void RobotinoMotionServer::controlLoop()
{
	setCmdVel(0, 0, 0);

	double dist_driven_x = fabs(dist_moved_x_); // current linear displacement in x axis
	double dist_driven_y = fabs(dist_moved_y_); // current linear displacement in y axis
	double dist_rotated_abs = fabs(dist_rotated_); // current angular displacement in phi axis

	double forward_goal_x_abs = fabs(forward_goal_x_);
	double forward_goal_y_abs = fabs(forward_goal_y_);
	double rotation_goal_abs = fabs(rotation_goal_);

	double distance_to_go_x = forward_goal_x_abs - dist_driven_x;
	double distance_to_go_y = forward_goal_y_abs - dist_driven_y;
	double distance_to_go_phi = rotation_goal_abs - dist_rotated_abs;

	feedback_.state.d_x = distance_to_go_x;
	feedback_.state.d_y = distance_to_go_y;
	feedback_.state.d_phi = distance_to_go_phi;

	double dist_driven = sqrt(pow(dist_driven_x, 2) + pow(dist_driven_y, 2));

	double dist_res = sqrt(pow(forward_goal_x_abs, 2) + pow(forward_goal_y_abs, 2)); // resultant linear displacement
	double ang_res = atan(forward_goal_y_abs / forward_goal_x_abs); // resultant direction
	double ang_res_real = atan(forward_goal_y_/forward_goal_x_);

	double vel_x = 0; // linear velocity in x axis
	double vel_y = 0; // linear velocity in y axis
	double vel_phi = 0; // angular velocity in phi axis

	double VEL;
	double VEL_TANG = 0.3;
	double VEL_ANG;
	
	switch (movement_type_)
	{
		case TRANSLATIONAL:
			if (dist_driven_x < forward_goal_x_abs || dist_driven_y < forward_goal_y_abs)
			{	
				
				if((dist_driven >= 0) && (dist_driven <= percentage_ * dist_res))
				{
					VEL = sqrt(pow(min_linear_vel_, 2) + 2 * linear_acc_ * dist_driven);
				
				}
				else if((dist_driven > percentage_ * dist_res) && (dist_driven <= (1 - percentage_)*dist_res))
				{
					VEL = sqrt(pow(min_linear_vel_, 2) + 2 * linear_acc_ * percentage_ * dist_res);
				}
				else
				{
					VEL = sqrt(pow(sqrt(pow(min_linear_vel_, 2) + 2 * linear_acc_ * percentage_ * dist_res), 2) - 2 * linear_acc_ * (dist_driven - ((1 - percentage_) * dist_res)));
				}
				/*if(dist_rotated_ < -5*PI/180)
				{
					vel_x = (VEL * cos(ang_res_real - dist_rotated_));
					vel_y = VEL * sin(ang_res_real - dist_rotated_);
				}
				else if(dist_rotated_ > 5*PI/180)
				{
					vel_x = (VEL * cos(ang_res_real - dist_rotated_));
					vel_y = VEL * sin(ang_res_real - dist_rotated_);
				}
				else
				{
					vel_x = (VEL * cos(ang_res_real));
					vel_y = VEL * sin(ang_res_real);
				}*/

				vel_x = VEL * cos(ang_res);
				vel_y = VEL * sin(ang_res);
				


				/*ROS_INFO("Moved (x, y) , VEL(x, y) =%f (%f, %f), (%f, %f), ang_res = %f, dist_rotated_ = %f ",
						dist_driven, dist_driven_x, dist_driven_y, vel_x, vel_y,
						ang_res, (float)((dist_rotated_ * 180) / PI));*/

			}
		break;
		case ROTATIONAL:
			if (dist_rotated_abs < rotation_goal_abs)
			{
				ROS_DEBUG("Rotated %f degrees", (dist_rotated_ * 180) / PI);

				if((dist_rotated_abs >= 0) && (dist_rotated_abs <= percentage_ * rotation_goal_abs))
				{
					VEL_ANG = sqrt(pow(min_angular_vel_, 2) + 2 * angular_acc_ * dist_rotated_abs);
				}
				else if((dist_rotated_abs > percentage_ * rotation_goal_abs) && (dist_rotated_abs <= (1 - percentage_)*rotation_goal_abs))
				{
					VEL_ANG = sqrt(pow(min_angular_vel_, 2) + 2 * angular_acc_ * percentage_ * rotation_goal_abs);
				}
				else
				{
					VEL_ANG = sqrt(pow(sqrt(pow(min_angular_vel_, 2) + 2 * angular_acc_ * percentage_ * rotation_goal_abs), 2) - 2 * angular_acc_ * (dist_rotated_abs - ((1 - percentage_) * rotation_goal_abs)));
				}


				vel_phi = VEL_ANG;

				/*ROS_INFO("RotGOAL = %f, Moved(phi) = %f, VEL_ANG = %f)", rotation_goal_abs, dist_rotated_abs, VEL_ANG);*/

			}
		break;
/*		case TRANSLATIONAL_ROTATIONAL:	
			if (dist_driven_x < forward_goal_x_abs || dist_driven_y < forward_goal_y_abs || dist_rotated_abs < rotation_goal_abs)
			{
				ROS_DEBUG("Moved (x, y) = (%f, %f) and rotated %f degrees", dist_driven_x, dist_driven_y, (dist_rotated_ * 180) / PI);
				
				double d, alpha, phi;
				d = sqrt(pow(forward_goal_x_, 2) + pow(forward_goal_y_, 2));
				if (forward_goal_x_ != 0)
					alpha = atan(forward_goal_y_ / forward_goal_x_);
				else 
					alpha = sign(forward_goal_y_) * PI / 2;
				phi = rotation_goal_;
				ROS_INFO("d = %f, alpha = %f and phi = %f", d, (alpha * 180) / PI, (phi * 180) / PI);
				
				double V_min_temp, omega_min_temp, V_min, omega_min;				
				V_min_temp = .05 * VEL;
				omega_min_temp = .05 * VEL_ANG;
				if (phi != 0)
					V_min = omega_min_temp * d / phi;
				else
					V_min = V_min_temp;
				if (d != 0)
					omega_min = V_min_temp * phi / d;
				else 
					omega_min = omega_min_temp;
				if (V_min < V_min_temp)
					V_min = V_min_temp;
				else 
					omega_min = omega_min_temp;
				ROS_INFO("V_min = %f and omega_min = %f", V_min, (omega_min * 180) / PI);

				double V_max_temp, omega_max_temp, V_max, omega_max;
				V_max_temp = VEL;
				omega_max_temp = VEL_ANG;
				if (phi != 0)
					V_max = omega_max_temp * d / phi;	
				else
					V_max = V_max_temp;
				if (d != 0)
					omega_max = V_max_temp * phi / d;
				else
					omega_max = omega_max_temp;
				if (V_max > V_max_temp)
					V_max = V_max_temp;
				else 
					omega_max = omega_max_temp;
				ROS_INFO("V_max = %f and omega_max = %f", V_max, (omega_max * 180) / PI);

				double percentage, K, kapa;
				percentage = 20;
				if (d != 0)
					K = (V_min - V_max) / ((percentage / 100) * (1 - percentage / 100) * pow(d, 2));
				else 
					K = 0;
				if (phi != 0)				
					kapa = (omega_min - omega_max) / ((percentage / 100) * (1 - percentage / 100) * pow(phi, 2));
				else 
					kapa = 0;
				ROS_INFO("p% = %f%, K = %f and kapa = %f", percentage, K, kapa);
 
				double s, theta;
				s = sqrt(pow(dist_moved_x_, 2) + pow(dist_moved_y_, 2));
				theta = dist_rotated_;
				ROS_INFO("s = %f and theta = %f", s, (theta * 180) / PI);
				
				double vel, omega;
				if (d == 0)
					vel = 0;				
				else if (s <= d * percentage / 100 || s >= d * (1 - percentage / 100))
					vel = K * s * (s - d) + V_min;
				else 
					vel = V_max;
				if (phi == 0)
					omega = 0;
				else if (theta <= phi * percentage / 100 || theta >= phi * (1 - percentage / 100))
					omega = kapa * theta * (theta - phi) + omega_min;
				else 
					omega = omega_max;
				ROS_INFO("vel = %f and omega = %f", vel, (omega * 180) / PI);				

				vel_x = vel * cos(alpha - theta);
				vel_y = vel * sin(alpha - theta);
				vel_phi = omega;
				ROS_INFO("vel_x = %f, vel_y = %f, vel_phi = %f)", vel_y, vel_x, (vel_phi * 180) / PI);
*/
				/*if (rotation_goal_ >=0)
				{
					if (forward_goal_x_ > 0)
					{
						vel_y = -VEL * sin(dist_rotated_abs + ang_res);
					}
					else
					{
						vel_y = VEL * sin(dist_rotated_abs + ang_res);
					}
					if (forward_goal_y_ > 0)
					{
						vel_x = -VEL * cos(dist_rotated_abs + ang_res);
					}
					else
					{
						vel_x = VEL * cos(dist_rotated_abs + ang_res);
					}
					vel_phi = rotation_goal_abs * VEL / dist_res;
				}
				else
				{
					if (forward_goal_x_ >= 0)
					{
						vel_y = VEL * sin(dist_rotated_abs + ang_res);
					}
					else
					{
						vel_y = -VEL * sin(dist_rotated_abs + ang_res);
					}
					if (forward_goal_y_ >= 0)
					{
						vel_x = VEL * cos(dist_rotated_abs + ang_res);
					}
					else
					{
						vel_x = -VEL * cos(dist_rotated_abs + ang_res);
					}
					vel_phi = rotation_goal_abs * VEL / dist_res;
				}
				*/
//			}	
//		break;
		case TANGENT:
			/*ROS_INFO("dist_driven_x = %f , forward_goal_x_abs = %f, dist_driven_y = %f, forward_goal_y_abs = %f, dist_rotated_abs = %f, rotation_goal_abs = %f", dist_driven_x, forward_goal_x_abs, dist_driven_y, forward_goal_y_abs, dist_rotated_abs, rotation_goal_abs);*/
			if (dist_driven_x < forward_goal_x_abs || dist_driven_y < forward_goal_y_abs || dist_rotated_abs < rotation_goal_abs)
			{
				double radius = .5 * sqrt(pow(forward_goal_x_abs, 2) + pow(forward_goal_y_abs, 2)) / sin(rotation_goal_abs / 2);				
	
				dist_res = rotation_goal_abs * 	radius;	
				dist_driven = dist_rotated_abs * radius;		

/*				if((dist_driven >= 0) && (dist_driven <= percentage * dist_res))
				{
					VEL = sqrt(pow(min_linear_vel, 2) + 2 * linear_acc * dist_driven);
				
				}
				else if((dist_driven > percentage * dist_res) && (dist_driven <= (1 - percentage)*dist_res))
				{
					VEL = sqrt(pow(min_linear_vel, 2) + 2 * linear_acc * percentage * dist_res);
				}
				else
				{
					VEL = sqrt(pow(sqrt(pow(min_linear_vel, 2) + 2 * linear_acc * percentage * dist_res), 2) - 2 * linear_acc * (dist_driven - ((1 - percentage) * dist_res)));
				}
*/

				vel_x = VEL_TANG * cos(ang_res);
				
				/*ROS_INFO("Moved (x, y) = (%f, %f) and roteated %f degrees , vel_x = %f", dist_driven_x, dist_driven_y, (dist_rotated_ * 180) / PI, vel_x);*/

/*				double radius = .5 * sqrt(pow(forward_goal_x_abs, 2) + pow(forward_goal_y_abs, 2)) / sin(rotation_goal_abs / 2);*/

				//vel_x = min_linear_vel;
				vel_y = 0;
				vel_phi = vel_x / radius;
			}	
		break;
		default:
			setCmdVel(0, 0, 0);
			return;
	}

	if (forward_goal_x_ < 0)
		vel_x = -vel_x;
	if (forward_goal_y_ < 0)
		vel_y = -vel_y;

	if(rotation_goal_ < 0)
		vel_phi = -vel_phi;

	if (vel_x == 0 && vel_y == 0 && vel_phi == 0)
		state_ = FINISHED;
	else
		setCmdVel(vel_x, vel_y, vel_phi);
}

bool RobotinoMotionServer::acceptNewGoal( const robotino_motion::MotionGoalConstPtr& goal )
{
	if (state_ == PROCESSING)
	{
		ROS_INFO("Processing Image!!!" );
		return false;
	}
	if(!odom_set_)
	{
		ROS_ERROR("Odometry not initialized!!!");
		return false;
	}
	else
	{
		forward_goal_x_ = goal->move_x;
		forward_goal_y_ = goal->move_y;
		rotation_goal_ = goal->move_phi * PI / 180;

		switch(goal->movement_type)
		{
			case 0: 
				movement_type_ = TRANSLATIONAL;
				break;
			case 1: 
				movement_type_ = ROTATIONAL;
				break;
			case 2: 
				movement_type_ = TRANSLATIONAL_ROTATIONAL;
				break;
			case 3: 
				movement_type_ = TANGENT;
				break;
			default:
				ROS_ERROR("Invalid movement_type: %d", goal->movement_type); 
				return false;
		}

		switch(goal->task_type)
		{
			case 0: 
				task_type_ = ALIGN;
				break;
			case 1: 
				task_type_ = MOVE;
				break;
			case 2: 
				task_type_ = COUNT;
				break;
			case 3: 
				task_type_ = FOLLOW;
				break;
			default:
				ROS_ERROR("Invalid task_type: %d", goal->task_type); 
				return false;
		}

		switch(goal->interruption_condition)
		{
			case 0: 
				interruption_condition_ = MOVED_DISTANCE;
				break;
			case 1: 
				interruption_condition_ = HIGH_OPTICAL_SIGNAL;
				break;
			case 2: 
				interruption_condition_ = LOW_OPTICAL_SIGNAL;
				break;
			case 3: 
				interruption_condition_ = HIGH_INDUCTIVE_SIGNAL;
				break;
			case 4: 
				interruption_condition_ = LOW_INDUCTIVE_SIGNAL;
				break;
			case 5: 
				interruption_condition_ = CAMERA;
				break;
			case 6: 
				interruption_condition_ = OBSTACLE;
				break;
			case 7: 
				interruption_condition_ = BUMPER;
				break;
			case 8: 
				interruption_condition_ = TIME;
				break;
			case 9: 
				interruption_condition_ = SECURED_INFRARED_SIGNAL;
				break;
			default:
				ROS_ERROR("Invalid interruption_condition: %d", goal->interruption_condition); 
				return false;
		}

		switch(goal->alignment_device)
		{
			case 0: 
				alignment_device_ = NONE;
				break;
			case 1: 
				alignment_device_ = INFRARED;
				break;
			case 2: 
				alignment_device_ = OPTICAL;
				break;
			case 3: 
				alignment_device_ = INDUCTIVE;
				break;
			case 4: 
				alignment_device_ = CAMERAAL;
				break;
			case 5: 
				alignment_device_ = ULTRASONIC;
				break;
			case 6: 
				alignment_device_ = COMPASS;
				break;			
			default:
				ROS_ERROR("Invalid alignment_device: %d", goal->alignment_device); 
				return false;
		}


		/*ROS_INFO( "Motion execution start: (x[m], y[m], phi[deg] ): (%f, %f, %f)",
				forward_goal_x_, forward_goal_y_, ((rotation_goal_ * 180) / PI ));*/

		start_x_ = curr_x_;
		start_y_ = curr_y_;
		start_phi_ = curr_phi_;
		dist_moved_x_ = 0.0;
		dist_moved_y_ = 0.0;
		dist_rotated_ = 0.0;

		if( fabs( forward_goal_x_ ) > 0.02 || fabs( forward_goal_y_ ) > 0.02 || fabs( rotation_goal_ ) > 0.01 )
		{
			state_ = MOVING;
		}
		else
		{
			state_ = FINISHED;
		}

		return true;
	}
}

void RobotinoMotionServer::readParameters( ros::NodeHandle& n)
{
	n.param( "min_linear_vel", min_linear_vel_, 0.2 );

	n.param( "max_linear_vel", max_linear_vel_, 0.8 );

	n.param( "linear_acc", linear_acc_, 0.1 );
	
	n.param( "min_angular_vel", min_angular_vel_, 0.5 );

	n.param( "max_angular_vel", max_angular_vel_, 0.8 );

	n.param( "angular_acc", angular_acc_, 0.1 );

	n.param( "percentage", percentage_, 0.4);

}

bool RobotinoMotionServer::getProduct(robotino_motion::GetProduct::Request &req, robotino_motion::GetProduct::Response &res)
{
	state_ = PROCESSING;	
	setAchievedGoal(false);
	int nframes_no_puck_ = 0;
	res.succeed = true;
	return true;
}

void RobotinoMotionServer::controlImageProcessing()
{
	//ROS_INFO("no puck=%d", nframes_no_puck_);
	double vel_x = 0;
	double vel_y = 0;
	double vel_phi = 0;
	if (!is_loaded_ || nframes_no_puck_ > 5)
	{
		robotino_vision::FindObjects srv;
		srv.request.color = 0; // always ORANGE (PUCK)
		vector<float> distances;
		vector<float> directions;
		if (!find_objects_cli_.call(srv))
		{	
			ROS_ERROR("Puck not found!!!");
			return;
		}
		int num_products = srv.response.distances.size();
		distances = srv.response.distances;
		directions = srv.response.directions; 
		
		int closest_index = 0;

		if (num_products > 0)
		{
			nframes_no_puck_ = 0;
		
			for (int i = 0; i < num_products; i++)
			{
				if (distances.at(i) < distances.at(closest_index))
				{
					closest_index = i;
				}
			}

			double max_error_front = 40;
			double error_front = 0;
			error_front = distances.at(closest_index);
		
			if (error_front > max_error_front)
			{
				 error_front = max_error_front;
			}

			float tolerance_y = 0.1;
			float tolerance_x = 25;
			double error_abs = fabs(directions.at(closest_index));
			double K_error = .3;
			double K_error_front = .002;
		
			if (directions.at(closest_index) < -tolerance_y)
			{
				vel_x = 0;
				vel_y = K_error * error_abs;
				vel_phi = 0;
			}
			else if (directions.at(closest_index) > tolerance_y)
			{
				vel_x = 0;
				vel_y = -K_error * error_abs;
				vel_phi = 0;
			}
			else if (error_front > tolerance_x)
			{
				vel_x = K_error_front * error_front;
				vel_y = 0;
				vel_phi = 0;
			}
			else
			{
				vel_x = .05;
				vel_y = 0;
				vel_phi = 0;
			}
		}
		else
		{	
			nframes_no_puck_++;
		}
	}
	else 
	{	
		state_ = IDLE;
		setAchievedGoal(true);/*
		robotino_motion::MotionGoal goal;
		goal.move_x = 0;
		goal.move_y = 0;
		goal.move_phi = 180;
		goal.movement_type = 1; 
		execute(goal);*/
	}
	setCmdVel(vel_x, vel_y, vel_phi);
	cmd_vel_pub_.publish(cmd_vel_msg_);
}

bool RobotinoMotionServer::align(robotino_motion::Align::Request &req, robotino_motion::Align::Response &res)
{
	bool succeed = true;
	alignment_mode_ = req.mode;
	switch (req.mode)
	{
		case FRONT: 
			left_index_ = 8;
			right_index_ = 1;
			lateral_ = false;
			break;
		case RIGHT: 
			left_index_ = 6;
			right_index_ = 7;
			lateral_ = true;
			break;
		case LEFT: 
			left_index_ = 2;
			right_index_ = 3;
			lateral_ = true;
			break;
		case BACK: 
			left_index_ = 4;
			right_index_ = 5;
			lateral_ = false;
			break;
		default:
			ROS_ERROR("Invalid side paremeter: %d!!!", req.mode);
			succeed = false;
	}
	state_ = ALIGNING;
	setAchievedGoal(false);
	res.succeed = succeed;
	return succeed;
}

void RobotinoMotionServer::controlAlignment()
{
	float K = .9;
	float K_PHY = 10;
	float error_phy = left_ir_ - right_ir_;
	float phy_tolerance = 0.005;
	float min_tolerance = 0.25;
	float max_tolerance = 0.30;
	float mean_value = (fabs(left_ir_) + fabs(right_ir_)) / 2;
	float error_min = mean_value - min_tolerance;
	float error_max = mean_value - max_tolerance;
	double vel_x = 0;
	double vel_y = 0;
	double vel_phi = 0;
	switch (alignment_mode_)
	{
		case FRONT:
			//ROS_INFO("Alinging FRONT"); 
			if (error_max > 0)
			{
				vel_x = K * error_max;
				vel_y = 0;
				vel_phi = 0;
			}
			else if (error_min < 0)
			{
				vel_x = K * error_min;
				vel_y = 0;
				vel_phi = 0;
			}
			else if (fabs(left_ir_ - right_ir_) > phy_tolerance)
			{
				vel_phi = - K_PHY * (left_ir_ - right_ir_);
			}
			break;
		case RIGHT:
			//ROS_INFO("Alinging RIGHT");
			if (error_max > 0)
			{
				vel_x = 0;
				vel_y = -K * error_max;
				vel_phi = 0;
			}
			else if (error_min < 0)
			{
				vel_x = 0;
				vel_y = -K * error_min;
				vel_phi = 0;
			}
			else if (fabs(left_ir_ - right_ir_) > phy_tolerance)
			{
				vel_phi = - K_PHY * (left_ir_ - right_ir_);
			}
			break;
		case LEFT:
			//ROS_INFO("Alinging LEFT");
			if (error_max > 0)
			{
				vel_x = 0;
				vel_y = K * error_max;
				vel_phi = 0;
			}
			else if (error_min < 0)
			{
				vel_x = 0;
				vel_y = K * error_min;
				vel_phi = 0;
			}
			else if (fabs(left_ir_ - right_ir_) > phy_tolerance)
			{
				vel_phi = K_PHY * (left_ir_ - right_ir_);
			}
			break;
		case BACK:
			//ROS_INFO("Alinging BACK");
			if (error_max > 0)
			{
				vel_x = -K * error_max;
				vel_y = 0;
				vel_phi = 0;
			}
			else if (error_min < 0)
			{
				vel_x = -K * error_min;
				vel_y = 0;
				vel_phi = 0;
			}
			else if (fabs(left_ir_ - right_ir_) > phy_tolerance)
			{
				vel_phi = - K_PHY * (left_ir_ - right_ir_);
			}
	}
	setCmdVel(vel_x, vel_y, vel_phi);
	cmd_vel_pub_.publish(cmd_vel_msg_);
	if (vel_x == 0 && vel_y == 0 && vel_phi ==0)	
	{
		state_ = IDLE;
		setAchievedGoal(true);
	}
}

bool RobotinoMotionServer::setAchievedGoal(bool achieved_goal)
{
	robotino_motion::SetAchievedGoal srv;	
	srv.request.achieved_goal = achieved_goal;
	if (!set_achieved_goal_cli_.call(srv))
	{
		ROS_ERROR("Problems to set achieved_goal!!!");
		return false;
	}
	return srv.response.succeed;	
}
