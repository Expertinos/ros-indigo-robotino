/*
 * RobotinoNode.h
 *
 *  Created on: 15/07/2014
 *      Author: adrianohrl@unifei.edu.br
 */

#ifndef RobotinoCameraNode_H
#define RobotinoCameraNode_H

#include "ComROS.h"
#include "CameraROS.h"
//#include "robotino_vision/LampPostState.h"
//#include "robotino_vision/PuckState.h"
//#include "FindPuck.h"

#include <ros/ros.h>

class RobotinoVision
{
public:
	RobotinoVision();
	~RobotinoVision();

	bool spin();
private:
	ros::NodeHandle nh_;

	//ros::Publisher lamp_post_state_pub_;
	//ros::Publisher puck_state_pub_;

	std::string hostname_;
	int cameraNumber_;

	ComROS com_;
	CameraROS camera_;
	//FindPuck find_puck;

	void initModules();
	Mat getImage();
};

#endif /* RobotinoCameraNode_H */
