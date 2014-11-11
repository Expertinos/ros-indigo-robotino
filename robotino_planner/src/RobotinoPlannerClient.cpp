/*
 * RobotinoPlannerClient.cpp
 *
 *  Created on: 12.10.2014
 *      Author: adrianohrl@unifei.edu.br
 */

#include "RobotinoPlannerClient.h"

RobotinoPlannerClient::RobotinoPlannerClient():
	client_("planner", false)
{
	abort_srv_ = nh_.advertiseService("abort", &RobotinoPlannerClient::abort, this);
	start_srv_ = nh_.advertiseService("start_module", &RobotinoPlannerClient::start, this);

	goal_pub_ = nh_.advertise<robotino_motion::MotionGoal>("goal", 1, true);
	
	hold_patient_cli_ = nh_.serviceClient<robotino_motion::GetProduct>("get_product");
	align_cli_ = nh_.serviceClient<robotino_motion::Align>("align");
	achieved_goal_cli_ = nh_.serviceClient<robotino_motion::AchievedGoal>("achieved_goal");
	go_to_cli_ = nh_.serviceClient<robotino_leds::GoFromTo>("go_from_to");
	sinalize_end_cli_ = nh_.serviceClient<robotino_leds::SinalizeEnd>("sinalize_end");
	stop_transport_cli_ = nh_.serviceClient<robotino_leds::StopTransportation>("stop_transportation");
	get_products_list_cli_ = nh_.serviceClient<robotino_vision::GetProductsList>("get_products_list");
}

RobotinoPlannerClient::~RobotinoPlannerClient()
{
	abort_srv_.shutdown();
	start_srv_.shutdown();

	goal_pub_.shutdown();

	hold_patient_cli_.shutdown();
	align_cli_.shutdown();
	go_to_cli_.shutdown();
	sinalize_end_cli_.shutdown();
	stop_transport_cli_.shutdown();
	achieved_goal_cli_.shutdown();
	get_products_list_cli_.shutdown();
}

Color RobotinoPlannerClient::getOrder()
{
	Color order;
	robotino_vision::GetProductsList srv;
	if (!get_products_list_cli_.call(srv))
	{
		ROS_ERROR("Failure to get order!!!");
		srv.response.succeed = false;
		return NONE;
	}
	int size = srv.response.products.size();
	if (size < 1)
	{	
		ROS_ERROR("No order!!!");
		srv.response.succeed = false;
		return NONE;
	}
	vector<int> orders;
	for (int i = 0; i < size; i++)
	{
		orders.push_back(srv.response.products[i]);
	}
	switch (orders[0])
	{	
		case 1:
			ROS_INFO("YELLOW");
			order = YELLOW;
			break;
		case 2:
			ROS_INFO("BLUE");
			order = BLUE;
			break;
		case 3:
			ROS_INFO("GREEN");
			order = GREEN;
			break;
		case 4:
			ROS_INFO("RED");
			order = RED;
			break;
		default:
			ROS_INFO("NONE");
			order = NONE;
			srv.response.succeed = false;
			return NONE;
	}
	srv.response.succeed = true;
	return order;
}

bool RobotinoPlannerClient::holdPatient()
{
	robotino_motion::GetProduct srv;
	srv.request.product = 0; // Always ORANGE (PUCK)
	if (!hold_patient_cli_.call(srv))
	{
		ROS_ERROR("Unreached Patient!!!");
		return false;
	}
	while(!hasBeenAchieved())
	{}
	return srv.response.succeed;
}

bool RobotinoPlannerClient::setLeds(Color color)
{
	robotino_leds::GoFromTo srv;
	switch (color)
	{
		case RED:
			srv.request.departure_place = 3;
			srv.request.arrival_place = 4;
			break;
		case YELLOW:
			srv.request.departure_place = 4;
			srv.request.arrival_place = 5;
			break;
		case BLUE:
			srv.request.departure_place = 2;
			srv.request.arrival_place = 3;
			break;
		case GREEN:
			srv.request.departure_place = 2;
			srv.request.arrival_place = 5;
	}
	if (!go_to_cli_.call(srv))
	{
		ROS_ERROR("Falure to set LEDs!!!");
		return false;
	}
	srv.response.succeed = true;
	return true;
}

bool RobotinoPlannerClient::align(AlignmentMode mode)
{
	robotino_motion::Align srv;
	
	switch (mode)
	{
		case FRONT:
			srv.request.mode = 0;
			break;
		case RIGHT:
			srv.request.mode = 1;
			break;
		case LEFT:
			srv.request.mode = 2;
			break;
		case BACK:
			srv.request.mode = 3;
	}
	if (!align_cli_.call(srv))
	{
		ROS_ERROR("Not possible to align!!!");
		return false;
	}
	while(!hasBeenAchieved())
	{}
	ROS_INFO("Acabei de alinhar!!!");
	return srv.response.succeed;
}

void RobotinoPlannerClient::publishGoal(float move_x, float move_y, float move_phi, int movement_type)
{
	robotino_motion::MotionGoal goal;
	goal.move_x = move_x;
	goal.move_y = move_y;
	goal.move_phi = move_phi;
	goal.movement_type = movement_type;
	goal.task_type = 0;
	goal.interruption_condition = 0;
	goal.alignment_device = 0;
	goal_pub_.publish(goal);
	while(!hasBeenAchieved())
	{}
}

bool RobotinoPlannerClient::hasBeenAchieved()
{
	robotino_motion::AchievedGoal srv;
	if (!achieved_goal_cli_.call(srv))
	{
		ROS_ERROR("Achieved goal service has failed!!!");
		return false;
	}
	return srv.response.achieved_goal;
}

bool RobotinoPlannerClient::stopTransportation()
{
	robotino_leds::StopTransportation srv;
	if (!stop_transport_cli_.call(srv))
	{
		ROS_ERROR("Failure to stop transportation!!!");
		return false;
	}
	srv.response.succeed = true;
	return true;
}

bool RobotinoPlannerClient::start(robotino_planner::StartModule::Request &req, robotino_planner::StartModule::Response &res)
{
	/*bool succeed = false;
	robotino_planner::PlannerGoal goal;
	goal.module = req.module;

	if ((goal.module == 0 || goal.module == 1) && checkServer())
	{
		ROS_INFO("Sending goal (Module: %d)", goal.module == 0 );
		sendGoal(goal);
		succeed = true;
	}	
	res.hasStarted = succeed;
	return succeed;*/
	if (req.module != 1)
	{
		ROS_INFO("Só tem o módulo B implementado aki!!!");
		return false;
	}
	/*ros::ServiceClient move_cli = nh_.serviceClient<robotino_motion::MoveTo>("move_to");
	robotino_motion::MoveTo move_srv;
	move_srv.request.x = xB;
	move_srv.request.y = yB;
	move_srv.request.obstacle_code = 0;
	if(!move_pub_.call(move_srv))
	{
		ROS_ERROR("problema em mover para ler lista");
		return;
	}*/
	
	ROS_INFO("Starting Module B!!!");
	for(int i = 0; i < 6; i++)
	{
		ROS_INFO("----------begin");
		align(BACK);
		align(LEFT);
		publishGoal(0.7,0,0,0);
		Color order = getOrder();
		publishGoal(0,0,85,1);
		publishGoal(0.8,0,0,0);
		publishGoal(0,-0.55,0,0);
		align(RIGHT);
		align(BACK);
		int counter = 0;
		/*do 
		{
			order = getOrder();
			if (order == NONE)
			{
				counter = 0;			
			}
			counter++;
		}while (order == NONE);
		ROS_INFO("%d", counter);*/
		stopTransportation();
		if (i == 0)
		{
			order = GREEN;
		}
		else if (i == 1)
		{
			order = RED;
		}
		else if (i == 2)
		{	
			order = YELLOW;
		}
		else if (i == 3)
		{
			order = BLUE;
		}
		else
		{
			break;
		}
		if (order == GREEN)
		{
			movingGREEN();
		}
		else if (order == RED)
		{
			movingRED();
		}
		else if (order == YELLOW)
		{
			movingYELLOW();
		}
		else if (BLUE)
		{
			movingBLUE();
		}
	}
}


void RobotinoPlannerClient::movingGREEN()
{	
	ROS_INFO("----------GREEN");
	publishGoal(1.75,0,0,0);
	ROS_INFO("----------Pegando o paciente!!!");
	holdPatient();
	ROS_INFO("----------Setando os LEDs!!!");
	setLeds(GREEN);
	publishGoal(0,0,85,1);
	publishGoal(0,0.1,0,0);
	publishGoal(0,0,85,1);
	align(LEFT);
	publishGoal(1.4,0,0,0);
	align(LEFT);
	publishGoal(0,0,-85,1);
	align(BACK);
	align(LEFT); // chegou no lugar do home2
	ROS_INFO("----------Chegou no home 2!!!");
	publishGoal(0,-0.1,0,0);
	publishGoal(1,0,0,0);
	publishGoal(-0.2,0,0,0);// entregou
	ROS_INFO("----------Chegou no lugar de entrega!!!");
	stopTransportation();
	publishGoal(0,0,180,1);
	publishGoal(0,-1.1,0,0);
	align(RIGHT);
	publishGoal(-0.45,0,0,0);
	ROS_INFO("----------origin!!!");
}

void RobotinoPlannerClient::movingRED()
{
	ROS_INFO("----------RED");
	publishGoal(1.75,0,0,0);
	align(RIGHT);
	publishGoal(0,0,85,1);
	align(BACK);
	publishGoal(.5,0,0,0);
	publishGoal(0,-0.5,0,0);
	align(RIGHT);
	publishGoal(0.70,0,0,0);
	align(RIGHT);

	//procurar elevador e liga o led
	ROS_INFO("Elevador");
	Color elevator;
	int counter = 0;
	do 
	{
		elevator = getOrder();
		if (elevator == RED)
		{
			counter = 0;
			setLeds(RED);			
		}
		counter++;
	}while (counter > 10);
	ros::Duration d(2);
	d.sleep();
	stopTransportation();

	publishGoal(0.9,0,0,0);
	align(RIGHT);
	publishGoal(0,1,0,0);
	align(BACK);
	publishGoal(0,0.5,0,0);
	align(LEFT);
	align(BACK); // alinha na parte do meio ntes de passar pela porta
	publishGoal(0,-0.2,0,0);
	publishGoal(0.5,0,0,0);
	publishGoal(0,0,85,1);
	publishGoal(1.5,0,0,0);
	publishGoal(0,0,180,1);
	align(BACK);
	publishGoal(0,-.4,0,0);
	align(RIGHT);
	publishGoal(0.3,0,0,0);
	holdPatient(); ///////////////
	setLeds(RED); /////////
	align(RIGHT);
	publishGoal(0,0.4,0,0);
	publishGoal(1.1,0,0,0);
	publishGoal(0,0.5,0,0);
	publishGoal(0,0,85,1);
	align(LEFT);
	publishGoal(0,0,-85,1);
	publishGoal(0,0.4,0,0);
	align(LEFT);
	publishGoal(0.25,0,0,0);
	publishGoal(-0.25,0,0,0);//acabou de deixar o puck
	stopTransportation();
	align(LEFT);
	align(BACK);// alinhou depois de colocar o puck
	publishGoal(0,-0.6,0,0);
	publishGoal(0,0,-85,1);
	publishGoal(0,1.65,0,0);
	align(LEFT);
	publishGoal(-0.6,0,0,0);
	align(LEFT);
	align(BACK);
	publishGoal(1.25,0,0,0);
	align(LEFT);
	publishGoal(1.35,0,0,0);//atravessando elevador aberto
	align(LEFT);
	publishGoal(0,0,-85,1);
	publishGoal(0.6,0,0,0);
	publishGoal(0,0.6,0,0);
	align(LEFT);
	publishGoal(1.4,0,0,0);
	align(LEFT);
	publishGoal(0,-0.8,0,0);
	publishGoal(0,0,85,1);
	publishGoal(0,-1,0,0);
	align(RIGHT);
	publishGoal(-0.2,0,0,0);//leds
}

void RobotinoPlannerClient::movingYELLOW()
{
	ROS_INFO("YELLOW");
	publishGoal(1.75,0,0,0);
	align(RIGHT);
	publishGoal(0,0,85,1);
	align(BACK);
	publishGoal(0.7,0,0,0);
	publishGoal(0,-0.5,0,0);
	align(RIGHT);
	publishGoal(0.7,0,0,0);
	align(RIGHT);
	
	//procurar elevador e liga o led
	ROS_INFO("Elevador");
	Color elevator;
	int counter = 0;
	do 
	{
		elevator = getOrder();
		if (elevator == RED)
		{
			counter = 0;
			setLeds(RED);			
		}
		counter++;
	}while (counter > 10);
	ros::Duration d(2);
	d.sleep();
	stopTransportation();

	publishGoal(1.9,0,0,0);
	align(RIGHT);
	publishGoal(0,0,85,1);
	align(BACK);
	align(RIGHT);
	publishGoal(0,0.7,0,0);
	align(BACK);
	publishGoal(0.9,0,0,0);
	publishGoal(0,0,-85,1);
	//pega o puck e liga led
	ROS_INFO("----------Pegando o paciente!!!");
	holdPatient();
	ROS_INFO("----------Setando os LEDs!!!");
	setLeds(YELLOW);
	publishGoal(0,-0.2,0,0);
	publishGoal(0,0,-85,1);
	publishGoal(0.3,0,0,0);
	publishGoal(0,0.4,0,0);
	align(LEFT);
	publishGoal(0,0,-85,1);
	align(BACK);
	align(LEFT);
	align(BACK);
	publishGoal(1.25,0,0,0);
	align(LEFT);
	publishGoal(1.4,0,0,0);// atravessando elevador
	publishGoal(0,0,-85,1);
	align(BACK);
	publishGoal(0.6,0,0,0);
	publishGoal(0,0.6,0,0);
	align(LEFT);
	publishGoal(1.6,0,0,0);
	align(LEFT);
	publishGoal(0,0,-85,1);// esta no home 2
	align(BACK);
	align(LEFT);
	publishGoal(0,-0.1,0,0);
	publishGoal(1,0,0,0);
	publishGoal(-0.2,0,0,0); //acabou de deixar o puck
	stopTransportation();	
	publishGoal(0,0,180,1);
	publishGoal(0,-1.1,0,0);
	align(RIGHT);
	publishGoal(-0.45,0,0,0);
}

void RobotinoPlannerClient::movingBLUE()
{
	ROS_INFO("BLUE");
	publishGoal(1.75,0,0,0);
	align(RIGHT);
	//pega o puck e liga led
	ROS_INFO("----------Pegando o paciente!!!");
	holdPatient();
	ROS_INFO("----------Setando os LEDs!!!");
	setLeds(BLUE);
	publishGoal(0,0,85,1);
	align(BACK);
	publishGoal(0.5,0,0,0);
	publishGoal(0,-0.3,0,0);
	align(RIGHT);
	publishGoal(0.7,0,0,0);
	align(RIGHT);

	//procurar elevador e liga o led
	ROS_INFO("Elevador");
	Color elevator;
	int counter = 0;
	do 
	{
		elevator = getOrder();
		if (elevator == RED)
		{
			counter = 0;
			setLeds(RED);			
		}
		counter++;
	}while (counter > 10);
	ros::Duration d(2);
	d.sleep();
	stopTransportation();
	setLeds(BLUE);

	publishGoal(1,0,0,0);
	align(RIGHT);
	publishGoal(0,1.2,0,0);
	publishGoal(0,0,85,1);
	align(LEFT);
	publishGoal(0,0,-85,1);
	align(BACK);	
	publishGoal(0,0.5,0,0);
	align(LEFT);
	publishGoal(0.5,0,0,0);
	publishGoal(0,0,85,1);
	publishGoal(0.7,0,0,0);
	publishGoal(0,0,85,1);
	publishGoal(0.2,0,0,0);
	publishGoal(-0.2,0,0,0);//deixa puck
	stopTransportation();
	publishGoal(0,0,-85,1);
	publishGoal(-0.8,0,0,0);
	publishGoal(0,0.4,0,0);
	align(LEFT);
	publishGoal(-1.6,0,0,0);
	align(BACK);
	publishGoal(0,1.3,0,0);
	align(BACK);
	publishGoal(0.6,0,0,0);
	publishGoal(0,0.5,0,0);
	align(LEFT);
	publishGoal(1.6,0,0,0);
	align(LEFT);
	publishGoal(0,-0.65,0,0);
	publishGoal(0,0,85,1);
	publishGoal(0,-1,0,0);
	align(RIGHT);
	publishGoal(-0.5,0,0,0);//leds
}

bool RobotinoPlannerClient::checkServer()
{
	for( int i = 0; i < 5; ++i)
	{
		ros::spinOnce();
		if(client_.waitForServer(ros::Duration(1.0)))
		{
			ROS_INFO("Connected to the Planner Server!!!");
			return true;
		}
		else
		{
			ROS_INFO("Waiting for Planner Server!!!");
		}
	}
	ROS_ERROR("Planner Server is not running!!!");
	return false;
}

void RobotinoPlannerClient::spin()
{
	ros::Rate loop_rate(5);
	ros::Time start_time = ros::Time::now();

	while(nh_.ok())
	{
		if(client_.waitForResult(ros::Duration(1.0)))
		{
			ROS_INFO("Planner succeeded!!!");
			break;
		}
		else
		{
			ROS_INFO("Planner is being executed!!!");
		}
		/*if((ros::Time::now() - start_time).toSec() > max_time_)
		{
			ROS_INFO("Timeout: Aborting Planner");
			client_.cancelAllGoals();
			break;
		}*/

		ros::spinOnce();
		loop_rate.sleep();
	}
}

void RobotinoPlannerClient::sendGoal(const robotino_planner::PlannerGoal& goal)
{
	client_.sendGoal(goal);
	ROS_INFO("Goal sent!!!");
}

bool RobotinoPlannerClient::abort(robotino_planner::Abort::Request &req, robotino_planner::Abort::Response &res)
{
	//////
	return true;
}
