/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*
* For a discussion of this tutorial, please see:
* http://pr.willowgarage.com/wiki/navigation/Tutorials/SendingSimpleGoals
*********************************************************************/
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <actionlib/client/terminal_state.h>
#include <robotino_motion/StorePuckAction.h>
#include <robotino_motion/GrabPuckAction.h>
#include "robotino_vision/FindObjects.h"
#include "Colors.h"

#include <boost/thread.hpp>

#include <queue>
#include <stdio.h>
#include <iostream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

Color color_;
robotino_vision::FindObjects srv;
ros::ServiceClient find_objects_cli_;

void spinThread(){
  ros::spin();
}

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  ros::NodeHandle n;

  boost::thread spin_thread = boost::thread(boost::bind(&spinThread));

  MoveBaseClient ac("move_base", true);
  actionlib::SimpleActionClient<robotino_motion::StorePuckAction> ac_sp("store_puck", true);
  actionlib::SimpleActionClient<robotino_motion::GrabPuckAction> ac_gp("grab_puck", true);

  ac.waitForServer(); //will wait for infinite time
  ac_sp.waitForServer();
  ac_gp.waitForServer();

  move_base_msgs::MoveBaseGoal goal;
  robotino_motion::StorePuckGoal goal_sp;
  robotino_motion::GrabPuckGoal goal_gp;

  std::queue<move_base_msgs::MoveBaseGoal> queue_;

  if(queue_.empty())
  {
	  goal.target_pose.header.seq = 0;
	  goal.target_pose.pose.position.x = -2.009;
	  goal.target_pose.pose.position.y = 1.114;
	  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(1.640);
	  queue_.push(goal);

	  goal.target_pose.header.seq = 1;
	  goal.target_pose.pose.position.x = -1.200;
	  goal.target_pose.pose.position.y = 1.083;
	  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(1.583);
	  queue_.push(goal);

	  goal.target_pose.header.seq = 2;
	  goal.target_pose.pose.position.x = -0.551;
	  goal.target_pose.pose.position.y = 1.533;
	  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(1.643);
	  queue_.push(goal);

	  goal.target_pose.header.seq = 3;
	  goal.target_pose.pose.position.x = 0.110;
	  goal.target_pose.pose.position.y = 1.192;
	  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(1.661);
	  queue_.push(goal);

	  goal.target_pose.header.seq = 4;
	  goal.target_pose.pose.position.x = 0.988;
	  goal.target_pose.pose.position.y = 1.255;
	  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(1.624);
	  queue_.push(goal);

	  goal.target_pose.header.seq = 5;
	  goal.target_pose.pose.position.x = 0.908;
	  goal.target_pose.pose.position.y = 1.440;
	  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-1.501);
	  queue_.push(goal);

	  goal.target_pose.header.seq = 6;
	  goal.target_pose.pose.position.x = -0.009;
	  goal.target_pose.pose.position.y = 1.241;
	  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-1.498);
	  queue_.push(goal);

	  goal.target_pose.header.seq = 7;
	  goal.target_pose.pose.position.x = -0.567;
	  goal.target_pose.pose.position.y = 1.047;
	  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-1.513);
	  queue_.push(goal);

	  goal.target_pose.header.seq = 8;
	  goal.target_pose.pose.position.x = -1.276;
	  goal.target_pose.pose.position.y = 0.829;
	  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-1.495);
	  queue_.push(goal);

	  goal.target_pose.header.seq = 9;
	  goal.target_pose.pose.position.x = -1.918;
	  goal.target_pose.pose.position.y = 0.910;
	  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-1.571);
	  queue_.push(goal);

	  //std::cout<<"goal: "<<goal<<std::endl;
  }



  //we'll send a goal to the robot to move 2 meters forward

  while(!queue_.empty())
  {
	  //give some time for connections to register
	  sleep(2.0);
	  goal = queue_.front();

	  goal.target_pose.header.frame_id = "map";//"base_link";
	  goal.target_pose.header.stamp = ros::Time::now();

	  ac.sendGoal(goal);
	  std::cout<<"goal: "<<goal<<std::endl;
	  ac.waitForResult();
	  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Cheguei no pose desejado  :)");
	  else
		ROS_INFO("BUGOU!!!!  :(");

	  goal_gp.color = 4; //RED
	  ac_gp.sendGoal(goal_gp);
	  ac_gp.waitForResult();
	  if(ac_gp.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Grabei!!!  :)");
	  else
		ROS_INFO("Cor errada!!!!  :(");


	  srv.request.color = Colors::toCode(color_);
	  if (find_objects_cli_.call(srv))
	  {
		  ac.sendGoal(goal);
		  ac.waitForResult();
		  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO("Cheguei no pose desejado");
		  else
			ROS_INFO("BUGOU!!!!");

		  goal_sp.mode = 0;
		  ac_sp.sendGoal(goal_sp);
		  ac_sp.waitForResult();
		  if(ac_sp.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO("Storei!!!  :)");
		  else
			ROS_INFO("BUGOU Store!!!!  :(");
	  }

	  queue_.pop();
  }
  return 0;
}
