/*
 * RobotinoLeds.h
 *
 *  Created on: 07/10/2014
 *      Author: adrianohrl@unifei.edu.br
 */

#ifndef RobotinoLeds_H
#define RobotinoLeds_H

#include <ros/ros.h>
#include <vector>
#include "robotino_msgs/DigitalReadings.h"
#include "robotino_leds/GoFromTo.h"
#include "robotino_leds/TransportProduct.h"
#include "robotino_leds/Trigger.h"

/**
 * Red LED is located at port DO0
 * Blue LED is located at port DO1
 * Yellow LED is located at port DO2
 * Green LED is located at port DO3
 */
#define RED 0
#define BLUE 1
#define YELLOW 2
#define GREEN 3

typedef enum {NONE, TV, DVD, CELULAR, TABLET, NOTEBOOK} Product;
typedef enum {ORIGIN, SETOR_DE_CONTROLE, EXAMES, CENTRO_CIRURGICO, SETOR_DE_RECUPERACAO, SETOR_DE_SAIDA} Place;

class RobotinoLeds
{
public:
	RobotinoLeds();
	~RobotinoLeds();

	bool spin();

private:
	ros::NodeHandle nh_;
	ros::Publisher digital_pub_;
	ros::ServiceServer go_srv_;
	ros::ServiceServer sinalize_srv_;
	ros::ServiceServer stop_srv_;
	ros::ServiceServer transport_srv_;

	robotino_msgs::DigitalReadings digital_msg_;

	Product product_;
	Place departure_place_;
	Place arrival_place_;

	double frequency_;
	int size_;

	void publish();	
	bool goFromTo(robotino_leds::GoFromTo::Request &req, robotino_leds::GoFromTo::Response &res);
	bool sinalizeEnd(robotino_leds::Trigger::Request &req, robotino_leds::Trigger::Response &res);
	bool stopTransportation(robotino_leds::Trigger::Request &req, robotino_leds::Trigger::Response &res);
	bool transportProduct(robotino_leds::TransportProduct::Request &req, robotino_leds::TransportProduct::Response &res);
	bool sinalizeTransportation();
	bool sinalizeEndOfTask();
	bool toggleLed(int led);
	bool toggleLed(int led1, int led2);
	bool setLeds();
	bool setLeds(std::vector<bool> mask);
	bool resetLeds();
	bool resetLeds(std::vector<bool> mask);
	bool isLighting(int led);
	std::string placeToString(Place place);
};

#endif /* RobotinoLeds_H */
