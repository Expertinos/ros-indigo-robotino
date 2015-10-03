#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <robotino_motion/GrabPuckAction.h>

int main (int argc, char **argv)
{
	ros::init(argc, argv, "grab_client_node");

	// create the action client
	// true causes the client to spin its own thread
	actionlib::SimpleActionClient<robotino_motion::GrabPuckAction> ac("grab_puck", true);

	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time

	ROS_INFO("Action server started, sending goal.");
	// send a goal to the action
	robotino_motion::GrabPuckGoal goal;
	//goal.color = 0;  		//ORANGE
	goal.color = 1;  		//YELLOW
	//goal.color = 2;   		//BLUE
	//goal.color = 3;   		//GREEN
	//goal.color = 4;   		//RED
	//goal.color = 5;   		//BLACK
	//goal.color = -1;   		//NONE
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
