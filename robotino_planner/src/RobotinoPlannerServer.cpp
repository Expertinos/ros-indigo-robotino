/*
 * RobotinoPlannerServer.cpp
 *
 *  Created on: 12.10.2014
 *      Author: adrianohrl@unifei.edu.br
 */

#include "RobotinoPlannerServer.h"

RobotinoPlannerServer::RobotinoPlannerServer():
	server_(nh_, "planner", boost::bind(&RobotinoPlannerServer::execute, this, _1), false)
{
	go_srv_ = nh_.serviceClient<robotino_leds::GoFromTo>("go_from_to");
	move_cli_ = nh_.serviceClient<robotino_planner::MoveTo>("move_to");
	stop_srv_ = nh_.serviceClient<robotino_leds::StopTransportation>("stop_transportation");
	transport_srv_ = nh_.serviceClient<robotino_leds::TransportProduct>("transport_product");
	has_arrived_sub_ = nh_.subscribe("has_arrived", 1, &RobotinoPlannerServer::hasArrived, this);

	// Module A variables:	
	has_arrived_ = false;
	num_lists_ = 2;
	num_orders_ = 3;
	list_ = 0;
	order_ = 0;

	// Module B variables:
	
}

RobotinoPlannerServer::~RobotinoPlannerServer()
{
	go_srv_.shutdown();
	move_cli_.shutdown();
	stop_srv_.shutdown();
	transport_srv_.shutdown();
	has_arrived_sub_.shutdown();
}

void RobotinoPlannerServer::spin()
{
	ros::Rate loop_rate(5);
	ROS_INFO("Robotino Planner Server up and running");
	while(nh_.ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}

void RobotinoPlannerServer::execute(const robotino_planner::PlannerGoalConstPtr& goal)
{
	ros::Rate loop_rate(10);
	if(!acceptNewGoal(goal))
	{
		ROS_WARN("Goal not accepted");
		return;
	}
	while(nh_.ok())
	{
		if(server_.isPreemptRequested())
		{
			if(server_.isNewGoalAvailable())
			{
				if(!acceptNewGoal(server_.acceptNewGoal()))
				{
					return;
				}
			}
			else
			{
				ROS_INFO("Cancel request");
				server_.setPreempted();
				return;
			}
		}
		if (module_ == MODULE_A || module_ == MODULE_B)
		{
			controlLoop();
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	server_.setAborted(robotino_planner::PlannerResult(), "Aborting on the goal because the node has been killed.");
}

void RobotinoPlannerServer::controlLoop()
{
	float xB,yB, xB_coord, yB_coord;
	for( list_ = 0 ; list_ < 2 ; list_++ )
	{
		//move_to para pegar lista de pedido x
		int aux = 6;

		ros::ServiceClient get_area_cli = nh_.serviceClient<robotino_mapping::GetAreaCoordinates>("get_product_coordinates");
		robotino_mapping::GetAreaCoordinates get_area_srv;
		get_area_srv.request.module = 0; //Modulo A
		get_area_srv.request.area = aux; //Order 1
		if(!get_area_cli.call(get_area_srv))
		{
			ROS_ERROR("problema pegar coord da lista pedidos");
			return;
		}
		xB_coord = get_area_srv.response.x;
		yB_coord = get_area_srv.response.y;



		ros::ServiceClient trans_coord_cli = nh_.serviceClient<robotino_mapping::GetIndexes>("get_indexes");
		robotino_mapping::GetIndexes trans_coord_srv;
		trans_coord_srv.request.x = xB_coord;
		trans_coord_srv.request.y = yB_coord;
		if(!trans_coord_cli.call(get_area_srv))
		{
			ROS_ERROR("problema transf coord da lista pedidos para i, j");
			return;
		}
		xB = trans_coord_srv.response.i;
		yB = trans_coord_srv.response.j;



		ros::ServiceClient move_cli = nh_.serviceClient<robotino_motion::MoveTo>("move_to");
		robotino_motion::MoveTo move_srv;
		move_srv.request.x = xB;
		move_srv.request.y = yB;
		move_srv.request.obstacle_code = 0;
		if(!move_cli.call(move_srv))
		{
			ROS_ERROR("problema em mover para ler lista");
			return;
		}
		while(!move_srv.response.succeed)
		{}




		ros::ServiceClient products_list_cli = nh_.serviceClient<robotino_vision::GetProductsList>("get_products_list");
		robotino_vision::GetProductsList products_list_srv;
		if(!products_list_cli.call(products_list_srv))
		{
			ROS_ERROR("problema em pegar produtos da lista de pedidos");
			return;
		}
		int size_prod = products_list_srv.response.products.size();
		AreaA listaPedidos[size_prod];
		for(int i = 0; i < size_prod; i++)
		{
			switch(products_list_srv.response.products[i])
			{
				case 1:
						listaPedidos[i] = TV;
						break;
				case 2:
						listaPedidos[i] = DVD;
						break;
				case 3:
						listaPedidos[i] = CELULAR;
						break;
				case 4:
						listaPedidos[i] = TABLET;
						break;
				case 5:
						listaPedidos[i] = NOTEBOOK;
						break;
			}
		}

		vector<int> map;

		for(int y = 0; y < size_prod; y++)
		{
			int pedido = listaPedidos[y];
			get_area_srv.request.module = 0; //Modulo A
			get_area_srv.request.area = pedido; //Order 1
			if(!get_area_cli.call(get_area_srv))
			{
				ROS_ERROR("problema pegar coord da lista pedido do PEDIDO");
				return;
			}
			xB_coord = get_area_srv.response.x;
			yB_coord = get_area_srv.response.y;

			trans_coord_srv.request.x = xB_coord;
			trans_coord_srv.request.y = yB_coord;
			if(!trans_coord_cli.call(get_area_srv))
			{
				ROS_ERROR("problema transf coord da lista pedidos para i, j do PEDIDO");
				return;
			}
			xB = trans_coord_srv.response.i;
			yB = trans_coord_srv.response.j;

			int obstacle;
			//define obstacle_code
			switch(pedido)
			{
				case TV:
					obstacle = 1;
					break;
				case DVD:
					obstacle = 2;
					break;
				case CELULAR:
					obstacle = 3;
					break;
				case TABLET:
					obstacle = 4;
					break;
				case NOTEBOOK:
					obstacle = 5;
					break;
			}

			//chama moveTo para o pedido da vez
			move_srv.request.x = xB;
			move_srv.request.y = yB;
			move_srv.request.obstacle_code = obstacle;
			if(!move_cli.call(move_srv))
			{
				ROS_ERROR("problema em mover para PEDIDO");
				return;
			}
			while(move_srv.response.succeed == true)
			{}


			//APANHA UM PRODUTO
			ros::ServiceClient get_product_cli = nh_.serviceClient<robotino_motion::GetProduct>("get_product");
			robotino_motion::GetProduct get_product_srv;
			get_product_srv.request.product = obstacle; //Produto a pegar
			if(!get_product_cli.call(get_product_srv))
			{
				ROS_ERROR("problema pegar o PEDIDO");
				return;
			}
			//Fica preso enquanto nao completa o movimento
			while(!get_product_srv.response.succeed)
			{}

			//SINALIZA TAL PRODUTO
			ros::ServiceClient transport_product_cli = nh_.serviceClient<robotino_leds::TransportProduct>("transport_product");
			robotino_leds::TransportProduct transport_product_srv;
			transport_product_srv.request.product = obstacle; //Produto a pegar
			if(!transport_product_cli.call(transport_product_srv))
			{
				ROS_ERROR("problema em ascender led do PEDIDO");
				return;
			}


			//VAI PARA AREA ONDE FOI FEITO O PEDIDO Y
			get_area_srv.request.module = 0; //Modulo A
			get_area_srv.request.area = order_; //Order 1
			if(!get_area_cli.call(get_area_srv))
			{
				ROS_ERROR("problema pegar coord da lista pedido da VOLTA PARA ORDER");
				return;
			}
			xB_coord = get_area_srv.response.x;
			yB_coord = get_area_srv.response.y;

			trans_coord_srv.request.x = xB_coord;
			trans_coord_srv.request.y = yB_coord;
			if(!trans_coord_cli.call(get_area_srv))
			{
				ROS_ERROR("problema transf coord da lista pedidos para i, j da VOLTA PARA ORDER");
				return;
			}
			xB = trans_coord_srv.response.i;
			yB = trans_coord_srv.response.j;
		}



	}
















	/*if (module_ == MODULE_A)
	{
		
		if (list_ < num_lists_) // for loop structure
		{
			
			if (order_ < num_orders_) // for loop structure
			{
				
				order_++;
			}
			else
			{
				order_ = 0;
			}
			list_++;
		}
			
	}
	else if (module_ == MODULE_B)
	{
		
	}*/
}

bool RobotinoPlannerServer::acceptNewGoal(const robotino_planner::PlannerGoalConstPtr& goal)
{
	switch (goal->module)
	{
		case 0:
			module_ = MODULE_A;
			num_lists_ = 2;
			num_orders_ = 3;			
			list_ = 0;
			order_ = 0;
			break;
		case 1:
			module_ = MODULE_B;
			break;
		default:
			ROS_ERROR("Invalid Modulo: %d", goal->module);
	}
}

void RobotinoPlannerServer::hasArrived(const std_msgs::BoolConstPtr& msg)
{
	has_arrived_ = msg->data;
}

/*
int RobotinoPlannerServer::getProductCode(Product product)
{
	int product_code = 0;
	switch (product)
	{
		case NONE:
			product_code = 0;
			break;
		case TV:
			product_code = 1;
			break;
		case DVD:
			product_code = 2;
			break;
		case CELULAR:
			product_code = 3;
			break;
		case TABLET:
			product_code = 4;
			break;
		case NOTEBOOK:
			product_code = 5;
	}
	return product_code;
}

int RobotinoPlannerServer::getPlaceCode(Place place)
{
	int place_code = 0;
	switch (place)
	{
		case ORIGIN:
			place_code = 0;
			break;
		case SETOR_DE_CONTROLE:
			place_code = 1;
			break;
		case EXAMES:
			place_code = 2;
			break;
		case CENTRO_CIRURGICO:
			place_code = 3;
			break;
		case SETOR_DE_RECUPERACAO:
			place_code = 4;
			break;
		case SETOR_DE_SAIDA:
			place_code = 5;
	}
	return place_code;
}*/
