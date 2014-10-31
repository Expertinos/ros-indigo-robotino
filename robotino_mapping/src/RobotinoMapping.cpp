/*
 * RobotinoMapping.cpp
 *
 *  Created on: 13/10/2014
 *      Author: adrianohrl@unifei.edu.br
 */

#include "RobotinoMapping.h"

RobotinoMapping::RobotinoMapping()
{
	get_coordinates_srv_ = nh_.advertiseService("get_coordinates", &RobotinoMapping::getCoordinates, this);
	get_indexes_srv_ = nh_.advertiseService("get_indexes", &RobotinoMapping::getIndexes, this);
	get_map_srv_ = nh_.advertiseService("get_map", &RobotinoMapping::getMap, this);
	get_area_coordinates_srv_ = nh_.advertiseService("get_product_coordinates", &RobotinoMapping::getAreaCoordinates, this);
	set_map_srv_ = nh_.advertiseService("set_map", &RobotinoMapping::setMap, this);

	nh_.param<string>("user_name", user_name_, "adriano");
	maps_path_ = "/home/" + user_name_ + "/catkin_ws/src/robotino/robotino_mapping/maps/";
	tv_map_ = readMap("tv_yellow.bmp");
	dvd_map_ = readMap("dvd_blue.bmp");
	celular_map_ = readMap("celular_green.bmp");
	tablet_map_ = readMap("tablet_red.bmp");
	notebook_map_ = readMap("notebook_black.bmp");
	namedWindow(MAP_WINDOW);
}

RobotinoMapping::~RobotinoMapping()
{
	get_indexes_srv_.shutdown();
	get_map_srv_.shutdown();
	get_area_coordinates_srv_.shutdown();
	set_map_srv_.shutdown();	
	destroyWindow(MAP_WINDOW);
}

bool RobotinoMapping::spin()
{
	ros::Rate loop_rate(10);
	ROS_INFO("Robotino Mapping node up and running!!!");
	while(nh_.ok())
	{		
		ros::spinOnce();
		loop_rate.sleep();
	}
	return true;
}

std::vector<int> RobotinoMapping::readMap(const char* file_name)
{
	vector<int> map;
	Mat map_image = imread(maps_path_ + file_name, 1);
	if (!map_image.data)
	{
		ROS_ERROR("No image data!!!");
		return map;
	}
	maps_height_ = map_image.rows;
	maps_width_ = map_image.cols;
	ROS_INFO("tamanhos: (%d, %d)", maps_height_, maps_width_);
	for (int i = 0; i < maps_height_ ; i++)
	{
		for (int j = 0; j < maps_width_; j++)
		{				
			Vec3b pixel = map_image.at<Vec3b>(i, j);
			map.push_back(1 - pixel.val[0] / 255);
		}
ROS_INFO(" %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d", map.at(i * maps_height_ + 0), map.at(i * maps_height_ + 1), map.at(i * maps_height_ + 2), map.at(i * maps_height_ + 3), map.at(i * maps_height_ + 4), map.at(i * maps_height_ + 5), map.at(i * maps_height_ + 6), map.at(i * maps_height_ + 7), map.at(i * maps_height_ + 8), map.at(i * maps_height_ + 9), map.at(i * maps_height_ + 10), map.at(i * maps_height_ + 11), map.at(i * maps_height_ +12), map.at(i * maps_height_ + 13), map.at(i * maps_height_ + 14), map.at(i * maps_height_ + 15), map.at(i * maps_height_ + 16));
	}
	return map;
}


bool RobotinoMapping::getCoordinates(robotino_mapping::GetCoordinates::Request &req, robotino_mapping::GetCoordinates::Response &res)
{
	Coordinates coordinates = convertIndexesToCoordinates(req.i, req.i);
	res.x = coordinates.x;
	res.y = coordinates.y;
	return true;
}

bool RobotinoMapping::getIndexes(robotino_mapping::GetIndexes::Request &req, robotino_mapping::GetIndexes::Response &res)
{
	Indexes indexes = convertCoordinatesToIndexes(req.x, req.y);
	res.i = indexes.i;
	res.j = indexes.j;
	return true;
}

bool RobotinoMapping::getMap(robotino_mapping::GetMap::Request &req, robotino_mapping::GetMap::Response &res)
{
	vector<int> chosen_map;
	switch(req.area)
	{
		case 1:
			chosen_map = tv_map_;
			break;
		case 2:
			chosen_map = dvd_map_;
			break;
		case 3:
			chosen_map = celular_map_;
			break;
		case 4:
			chosen_map = tablet_map_;
			break;
		case 5:
			chosen_map = notebook_map_;
			break;
		default:
			ROS_ERROR("Invalid area code: %d", req.area);
	}
	res.map.clear();
	for (int i = 0; i < maps_height_; i++)
	{
		for (int j = 0; j < maps_width_; j++)
		{
			res.map.push_back(chosen_map.at(i * maps_height_ + j));
			ROS_INFO("(%d,%d)=%d",i,j,res.map[i * maps_height_ + j]);
		}
	}
	return true;
}

bool RobotinoMapping::getAreaCoordinates(robotino_mapping::GetAreaCoordinates::Request &req, robotino_mapping::GetAreaCoordinates::Response &res)
{
	switch (req.module)
	{
		case 0:
			module_ = MODULE_A;
			break;
		case 1: 
			module_ = MODULE_B;
			break;
		default:
			ROS_ERROR("Invalid module code: %d", req.module);
			return false;
	}
	Point2f point;
	if (module_ == MODULE_A)
	{
		AreaA area;
		switch (req.area)
		{
			case 0:
				area = HOME;
				break;
			case 1:
				area = TV; 
				break;
			case 2:
				area = DVD; 
				break;
			case 3:
				area = CELULAR; 
				break;
			case 4:
				area = TABLET; 
				break;
			case 5:
				area = NOTEBOOK; 
				break;
			case 6:
				area = ORDER_1; 
				break;
			case 7:
				area = ORDER_2;
				break;
			default:
				ROS_ERROR("Invalid are code for module A: %d", req.area);
				return false;
		}
		point = getArea(area);
	}
	else
	{
		AreaB area;
		switch (req.area)
		{
			case 0:
				area = ORIGIN;
				break;
			case 1:
				area = ELEVATOR; 
				break;
			case 2:
				area = SETOR_DE_CONTROLE; 
				break;
			case 3:
				area = EXAMES; 
				break;
			case 4:
				area = CENTRO_CIRURGICO; 
				break;
			case 5:
				area = SETOR_DE_RECUPERACAO; 
				break;
			case 6:
				area = SETOR_DE_SAIDA; 
				break;
			default:
				ROS_ERROR("Invalid are code for module B: %d", req.area);
				return false;
		}
		point = getArea(area);
	}	
	res.x = point.x;
	res.y = point.y;	
	return true;
}

bool RobotinoMapping::setMap(robotino_mapping::SetMap::Request &req, robotino_mapping::SetMap::Response &res)
{
	bool succeed = true;
	vector<int> map = readMap(req.file_name.c_str());
	switch (req.product)
	{
		case 1:
			tv_map_ = map;
			break;
		case 2:
			dvd_map_ = map;
			break;
		case 3:
			celular_map_ = map;
			break;
		case 4:
			tablet_map_ = map; 
			break;
		case 5:
			notebook_map_ = map; 
			break;
		default:
			ROS_ERROR("Invalid product code: %d", req.product);
			succeed = false;
	}
	res.setted = succeed;
	return succeed;
}

cv::Point2f RobotinoMapping::getArea(AreaA area)
{
	Point2f point(0, 0);
	switch (area)
	{
		case HOME:
			point.x = 15;
			point.y = 7;
			break;
		case TV:
			point.x = 11;
			point.y = 7;
			break;
		case DVD:
			point.x = 9;
			point.y = 1;
			break;
		case CELULAR:
			point.x = 3;
			point.y = 5;
			break;
		case TABLET:
			point.x = 3;
			point.y = 9;
			break;
		case NOTEBOOK:
			point.x = 9;
			point.y = 15;
			break;
		case ORDER_1:
			point.x = 13;
			point.y = 1;
			break;
		case ORDER_2:
			point.x = 13;
			point.y = 15;
	}
	return point;
}

cv::Point2f RobotinoMapping::getArea(AreaB area)
{
	Point2f point(0, 0);
	switch (area)
	{
		case ORIGIN:
			point.x = 1;
			point.y = 1;
			break;
		case ELEVATOR:
			point.x = 2;
			point.y = 2;
			break;
		case SETOR_DE_CONTROLE:
			point.x = 3;
			point.y = 3;
			break;
		case EXAMES:
			point.x = 4;
			point.y = 4;
			break;
		case CENTRO_CIRURGICO:
			point.x = 5;
			point.y = 5;
			break;
		case SETOR_DE_RECUPERACAO:
			point.x = 6;
			point.y = 6;
			break;
		case SETOR_DE_SAIDA:
			point.x = 7;
			point.y = 7;
	}
	return point;
}

Coordinates RobotinoMapping::convertIndexesToCoordinates(Indexes indexes)
{
	return convertIndexesToCoordinates(indexes.i, indexes.j);
}

Coordinates RobotinoMapping::convertIndexesToCoordinates(int i, int j)
{
	Coordinates coordinates;
	coordinates.x = .5 * delta_x_ * i; // in centimeters
	coordinates.y = .5 * delta_y_ * j; // in centimeters
	return coordinates;
}

Indexes RobotinoMapping::convertCoordinatesToIndexes(Coordinates coordinates)
{
	return convertCoordinatesToIndexes(coordinates.x, coordinates.y);
}

Indexes RobotinoMapping::convertCoordinatesToIndexes(float x, float y)
{
	Indexes indexes;
	indexes.i = ceil(2 * x / delta_x_); 
	indexes.j = ceil(2 * y / delta_y_); 
	return indexes;	
}
