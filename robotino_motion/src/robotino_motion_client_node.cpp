/*
 * robotino_motion_client_node.cpp
 *
 *  Created on: 2014
 *      Author: expertinos.unifei@gmail.com
 */

#include "RobotinoMotionClient.h"

#define PI 3.14159

int main(int argc, char** argv)
{
	ros::init(argc, argv, "robotino_motion_client_node");
	ros::NodeHandle nh_;

	RobotinoMotionClient rmc;

	robotino_motion::MotionGoal goal;
	float max_time;

	switch(argc){
	case 2:
	{
		std::istringstream move_x( argv[1] );
		move_x >> goal.move_x;
		std::istringstream move_y( 0 );
		move_y >> goal.move_y;
		std::istringstream move_phi( 0 );
		move_phi >> goal.move_phi;
		std::istringstream movement_type( 0 );
		movement_type >> goal.movement_type;
		std::istringstream task_type( 0 );
		task_type >> goal.task_type;
		std::istringstream interruption_condition( 0 );
		interruption_condition >> goal.interruption_condition;
		std::istringstream alignment_device( 0 );
		alignment_device >> goal.alignment_device;	
		
		rmc.sendGoal(goal);
	}
	
	case 3:
	{
		std::istringstream move_x( argv[1] );
		move_x >> goal.move_x;
		std::istringstream move_y( argv[2] );
		move_y >> goal.move_y;
		std::istringstream move_phi( 0 );
		move_phi >> goal.move_phi;
		std::istringstream movement_type( 0 );
		movement_type >> goal.movement_type;
		std::istringstream task_type( 0 );
		task_type >> goal.task_type;
		std::istringstream interruption_condition( 0 );
		interruption_condition >> goal.interruption_condition;
		std::istringstream alignment_device( 0 );
		alignment_device >> goal.alignment_device;	
		
		rmc.sendGoal(goal);	
	}

	case 4:
	{
		std::istringstream move_x( argv[1] );
		move_x >> goal.move_x;
		std::istringstream move_y( argv[2] );
		move_y >> goal.move_y;
		std::istringstream move_phi( argv[3] );
		move_phi >> goal.move_phi;
		std::istringstream movement_type( 0 );
		movement_type >> goal.movement_type;
		std::istringstream task_type( 0 );
		task_type >> goal.task_type;
		std::istringstream interruption_condition( 0 );
		interruption_condition >> goal.interruption_condition;
		std::istringstream alignment_device( 0 );
		alignment_device >> goal.alignment_device;	
		
		rmc.sendGoal(goal);	
	}

	case 5:
	{
		std::istringstream move_x( argv[1] );
		move_x >> goal.move_x;
		std::istringstream move_y( argv[2] );
		move_y >> goal.move_y;
		std::istringstream move_phi( argv[3] );
		move_phi >> goal.move_phi;
		std::istringstream movement_type( argv[4] );
		movement_type >> goal.movement_type;
		std::istringstream task_type( 0 );
		task_type >> goal.task_type;
		std::istringstream interruption_condition( 0 );
		interruption_condition >> goal.interruption_condition;
		std::istringstream alignment_device( 0 );
		alignment_device >> goal.alignment_device;	
		
		rmc.sendGoal(goal);
	}

	case 6:
	{
		std::istringstream move_x( argv[1] );
		move_x >> goal.move_x;
		std::istringstream move_y( argv[2] );
		move_y >> goal.move_y;
		std::istringstream move_phi( argv[3] );
		move_phi >> goal.move_phi;
		std::istringstream movement_type( argv[4] );
		movement_type >> goal.movement_type;
		std::istringstream task_type( argv[5] );
		task_type >> goal.task_type;
		std::istringstream interruption_condition( 0 );
		interruption_condition >> goal.interruption_condition;
		std::istringstream alignment_device( 0 );
		alignment_device >> goal.alignment_device;	
		
		rmc.sendGoal(goal);
	}

	case 7:
	{
		std::istringstream move_x( argv[1] );
		move_x >> goal.move_x;
		std::istringstream move_y( argv[2] );
		move_y >> goal.move_y;
		std::istringstream move_phi( argv[3] );
		move_phi >> goal.move_phi;
		std::istringstream movement_type( argv[4] );
		movement_type >> goal.movement_type;
		std::istringstream task_type( argv[5] );
		task_type >> goal.task_type;
		std::istringstream interruption_condition( argv[6] );
		interruption_condition >> goal.interruption_condition;
		std::istringstream alignment_device( 0 );
		alignment_device >> goal.alignment_device;	
		
		rmc.sendGoal(goal);
	}

	case 8:
	{
		std::istringstream move_x( argv[1] );
		move_x >> goal.move_x;
		std::istringstream move_y( argv[2] );
		move_y >> goal.move_y;
		std::istringstream move_phi( argv[3] );
		move_phi >> goal.move_phi;
		std::istringstream movement_type( argv[4] );
		movement_type >> goal.movement_type;
		std::istringstream task_type( argv[5] );
		task_type >> goal.task_type;
		std::istringstream interruption_condition( argv[6] );
		interruption_condition >> goal.interruption_condition;
		std::istringstream alignment_device( argv[7] );
		alignment_device >> goal.alignment_device;	
		
		rmc.sendGoal(goal);
	}

}
	ros::spin();

	return 0;
}
