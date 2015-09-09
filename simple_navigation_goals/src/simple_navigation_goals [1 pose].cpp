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

#include <boost/thread.hpp>

#include <queue>
#include <stdio.h>
#include <iostream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void spinThread(){
  ros::spin();
}

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  ros::NodeHandle n;

  boost::thread spin_thread = boost::thread(boost::bind(&spinThread));

  MoveBaseClient ac("move_base", true);

  move_base_msgs::MoveBaseGoal goal;

  std::queue<move_base_msgs::MoveBaseGoal> queue_;

  if(queue_.empty())
  {
	  goal.target_pose.header.seq = 0;
	  goal.target_pose.pose.position.x = 0.568;
	  goal.target_pose.pose.position.y = 0.165;
	  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-2.478);
	  queue_.push(goal);

  }

  //we'll send a goal to the robot to move 2 meters forward

  while(!queue_.empty())
  {
	  //give some time for connections to register
	  sleep(10.0);
	  goal = queue_.front();

	  goal.target_pose.header.frame_id = "map";//"base_link";
	  goal.target_pose.header.stamp = ros::Time::now();

	  ROS_INFO("Sending goal");
	  ac.sendGoal(goal);
	  std::cout<<"goal: "<<goal<<std::endl;
	  ac.waitForResult();

	  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Cheguei no pose desejado");
	  else
		ROS_INFO("BUGOU!!!!");

	  queue_.pop();
  }
  return 0;
}
