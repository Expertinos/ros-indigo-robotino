/*
 * RobotinoNode.cpp
 *
 *  Created on: 15/07/2014
 *      Author: adrianohrl@unifei.edu.br
 */

#include "RobotinoVision.h"
#include <sstream>

RobotinoVision::RobotinoVision()
	: nh_("~")
{
	nh_.param<std::string>("hostname", hostname_, "0.0.0.0:12080");
	nh_.param<int>("cameraNumber", cameraNumber_, 0);

	//lamp_post_state_pub_ = nh_.advertise<robotino_vision::LampPostState>("/lamp_post/state", 10);
	//puck_state_pub_ = nh_.advertise<robotino_vision::PuckState>("/puck/state", 10);

	std::ostringstream os;
	os << "Camera" << cameraNumber_;
	com_.setName(os.str());

	initModules();
}

RobotinoVision::~RobotinoVision()
{
	//lamp_post_pub_.shutdown();
	//puck_state_.shutdown();
}

void RobotinoVision::initModules()
{
	com_.setAddress(hostname_.c_str());
	camera_.setComId(com_.id());
	camera_.setNumber(cameraNumber_);
	com_.connectToServer(false);
}

bool RobotinoVision::spin()
{
	ros::Rate loop_rate(30);
	while(nh_.ok())
	{
		ros::Time curr_time = ros::Time::now();
		camera_.setTimeStamp(curr_time);
		com_.processEvents();
		
		//find_puck.publish_obj_pos(camera_.getImage());
		//find_puck.calibrate(camera_.getImage());
		//find_puck.tresCores(camera_.getImage());

		ros::spinOnce();
		loop_rate.sleep();
	}
	return true;
}

Mat RobotinoVision::getImage()
{
	return camera_.getImage();
}
