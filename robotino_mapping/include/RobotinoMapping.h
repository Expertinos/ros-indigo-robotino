/*
 * RobotinoMapping.h
 *
 *  Created on: 13/10/2014
 *      Author: adrianohrl@unifei.edu.br
 */

#ifndef RobotinoMapping_H
#define RobotinoMapping_H

#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <vector>

#include "robotino_mapping/GetCoordinates.h"
#include "robotino_mapping/GetIndexes.h"
#include "robotino_mapping/GetMap.h"
#include "robotino_mapping/GetAreaCoordinates.h"
#include "robotino_mapping/SetMap.h"

using namespace cv;
using namespace std;

typedef enum {HOME, TV, DVD, CELULAR, TABLET, NOTEBOOK, ORDER_1, ORDER_2} AreaA;
typedef enum {ORIGIN, ELEVATOR, SETOR_DE_CONTROLE, EXAMES, CENTRO_CIRURGICO, SETOR_DE_RECUPERACAO, SETOR_DE_SAIDA} AreaB;
typedef enum {MODULE_A, MODULE_B} Module;

struct Coordinates
{
	double x;
	double y;
};

struct Indexes
{
	double i;
	double j;
};

static const string MAP_WINDOW = "Map";

class RobotinoMapping
{
public:
	RobotinoMapping();
	~RobotinoMapping();

	bool spin();

private:
	ros::NodeHandle nh_;
 	ros::ServiceServer get_coordinates_srv_;
 	ros::ServiceServer get_indexes_srv_;
	ros::ServiceServer get_map_srv_;
 	ros::ServiceServer get_area_coordinates_srv_;
	ros::ServiceServer set_map_srv_;
		
	Module module_;
	vector<int> tv_map_;
	vector<int> dvd_map_;
	vector<int> celular_map_;
	vector<int> tablet_map_;
	vector<int> notebook_map_;
	string user_name_;
	string maps_path_;
	float delta_x_;
	float delta_y_;
	int maps_height_;
	int maps_width_;

	vector<int> readMap(const char* file_name);
	bool getCoordinates(robotino_mapping::GetCoordinates::Request &req, robotino_mapping::GetCoordinates::Response &res);
	bool getIndexes(robotino_mapping::GetIndexes::Request &req, robotino_mapping::GetIndexes::Response &res);
	bool getMap(robotino_mapping::GetMap::Request &req, robotino_mapping::GetMap::Response &res);
	bool getAreaCoordinates(robotino_mapping::GetAreaCoordinates::Request &req, robotino_mapping::GetAreaCoordinates::Response &res);
	bool setMap(robotino_mapping::SetMap::Request &req, robotino_mapping::SetMap::Response &res);
	cv::Point2f getArea(AreaA area);
	cv::Point2f getArea(AreaB area);
	Coordinates convertIndexesToCoordinates(Indexes indexes);
	Coordinates convertIndexesToCoordinates(int i, int j);
	Indexes convertCoordinatesToIndexes(Coordinates coordinates);
	Indexes convertCoordinatesToIndexes(float x, float y);

};

#endif /* RobotinoMapping_H */
