#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <robotino_motion/AlignAction.h>

int main (int argc, char **argv)
{
	ros::init(argc, argv, "align_client_node");

	// create the action client
	// true causes the client to spin its own thread
	actionlib::SimpleActionClient<robotino_motion::AlignAction> ac("align", true);
	ROS_INFO("%d", argc);
	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time

	ROS_INFO("Action server started, sending goal.");
	// send a goal to the action
	robotino_motion::AlignGoal goal;
	goal.alignment_mode = 3; // BACK mode
	if (argc > 1)
	{
		goal.alignment_mode = atoi(argv[1]);
	}
	goal.distance_mode = 1; // NORMAL distance
	if (argc > 2)
	{
		goal.distance_mode = atoi(argv[2]);
	}
	ac.sendGoal(goal);

	//wait for the action to return
	bool finished_before_timeout = ac.waitForResult(ros::Duration(50.0));
	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s",state.toString().c_str());
	}
	else
	{
		ROS_INFO("Action did not finish before the time out.");
	}

	//exit
	return 0;
}
