/*
 * RobotinoLeds.cpp
 *
 *  Created on: 07/10/2014
 *      Author: adrianohrl@unifei.edu.br
 */

#include "RobotinoLeds.h"

RobotinoLeds::RobotinoLeds()
{
	nh_.param<double>("frequency", frequency_, 1.0);
	digital_pub_ = nh_.advertise<robotino_msgs::DigitalReadings>("set_digital_values", 1, true);
	go_srv_ = nh_.advertiseService("go_from_to", &RobotinoLeds::goFromTo, this);
	sinalize_srv_ = nh_.advertiseService("sinalize_end", &RobotinoLeds::sinalizeEnd, this);
	stop_srv_ = nh_.advertiseService("stop_transportation", &RobotinoLeds::stopTransportation, this);
	transport_srv_ = nh_.advertiseService("transport_product", &RobotinoLeds::transportProduct, this);
	product_ = NONE;
	departure_place_ = ORIGIN;
	arrival_place_ = SETOR_DE_CONTROLE;
	size_ = 8;
	digital_msg_.values.resize(size_);
	resetLeds();
	publish();
}

RobotinoLeds::~RobotinoLeds()
{
	digital_pub_.shutdown();
	go_srv_.shutdown();
	sinalize_srv_.shutdown();
	stop_srv_.shutdown();
	transport_srv_.shutdown();	
}

bool RobotinoLeds::spin()
{
	ros::Rate loop_rate(2 * frequency_);
	ROS_INFO("Robotino LED node is up and running!!!");
	while(nh_.ok())
	{
		if (product_ != NONE || departure_place_ != ORIGIN || arrival_place_ != SETOR_DE_CONTROLE)
		{
			sinalizeTransportation();
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return true;
}

void RobotinoLeds::publish()
{
	digital_msg_.stamp = ros::Time::now();
	digital_pub_.publish(digital_msg_);
}

bool RobotinoLeds::goFromTo(robotino_leds::GoFromTo::Request &req, robotino_leds::GoFromTo::Response &res)
{
	bool success = true;
	std::stringstream message;
	success = resetLeds();
	publish();
	switch (req.departure_place)
	{
		case 0:
			departure_place_ = ORIGIN;
			break;
		case 1:
			departure_place_ = SETOR_DE_CONTROLE;
			break;
		case 2:
			departure_place_ = EXAMES;
			break;
		case 3:
			departure_place_ = CENTRO_CIRURGICO;
			break;
		case 4:
			departure_place_ = SETOR_DE_RECUPERACAO;
			break;
		case 5:
			departure_place_ = SETOR_DE_SAIDA;
			break;
		default:
			message << "Invalid departure place code: " << req.departure_place << "!!!";
			ROS_ERROR("Invalid departure place code: %d!!!", req.departure_place);
			success = false;
	}
	switch (req.arrival_place)
	{
		case 0:
			arrival_place_ = ORIGIN;
			break;
		case 1:
			arrival_place_ = SETOR_DE_CONTROLE;
			break;
		case 2:
			arrival_place_ = EXAMES;
			break;
		case 3:
			arrival_place_ = CENTRO_CIRURGICO;
			break;
		case 4:
			arrival_place_ = SETOR_DE_RECUPERACAO;
			break;
		case 5:
			arrival_place_ = SETOR_DE_SAIDA;
			break;
		default:
			message << "Invalid arrival place code: " << req.arrival_place << "!!!";
			ROS_ERROR("Invalid arrival place code: %d!!!", req.arrival_place);
			ROS_ERROR("%s", message.str().c_str());
			success = false;
	}
	if (success) 
	{
		message << "Going from " << placeToString(departure_place_) << " to " << placeToString(arrival_place_) << "!!!";
		ROS_DEBUG("%s", message.str().c_str());
	}
	res.success = success;
	res.message = message.str();
	return success;
}

bool RobotinoLeds::sinalizeEnd(robotino_leds::Trigger::Request &req, robotino_leds::Trigger::Response &res)
{
	ROS_DEBUG("Sinalizing End!!!");
	bool success = false;
	product_ = NONE;
	departure_place_ = ORIGIN;
	arrival_place_ = SETOR_DE_CONTROLE;
	success = sinalizeEndOfTask();
	res.success = success;
	res.message = "Sinalized!!!";
	if (!success) 
	{
		res.message = "Unable to sinalize!!!";
	}
	return success;
}

bool RobotinoLeds::stopTransportation(robotino_leds::Trigger::Request &req, robotino_leds::Trigger::Response &res)
{
	ROS_DEBUG("Stopping Transportation!!!");
	bool success = true;
	product_ = NONE;
	departure_place_ = ORIGIN;
	arrival_place_ = SETOR_DE_CONTROLE;
	res.success = success;
	res.message = "Transportation has been aborted!!!";
	if (!success)
	{
		res.message = "Unable to stop transportation!!!";
	}
	return success;
}

bool RobotinoLeds::transportProduct(robotino_leds::TransportProduct::Request &req, robotino_leds::TransportProduct::Response &res)
{
	bool success = true;
	std::stringstream message;
	success = resetLeds();
	publish();
	switch (req.product)
	{
		case 1:
			product_ = TV;
			message << "Transporting a TV!!!";
			break;
		case 2:
			product_ = DVD;
			message << "Transporting a DVD!!!";
			break;
		case 3:
			product_ = CELULAR;
			message << "Transporting a CELULAR!!!";
			break;
		case 4:
			product_ = TABLET;
			message << "Transporting a TABLET!!!";
			break;
		case 5:
			product_ = NOTEBOOK;
			message << "Transporting a NOTEBOOK!!!";
			break;
		default:
			message << "Invalid requested product: " << req.product <<"!!!";
			ROS_ERROR("Invalid requested product: %d!!!", req.product);
			success = false;
	}	
	if (success) 
	{
		ROS_DEBUG("%s", message.str().c_str());
	}
	res.success = success;
	res.message = message.str();
	return success;
}

bool RobotinoLeds::sinalizeTransportation()
{
	bool success = true;
	switch (product_)
	{
		case NONE:
			success = false;
			break;
		case TV:
			success = toggleLed(YELLOW, BLUE);
			break;
		case DVD:
			success = toggleLed(BLUE, GREEN);
			break;
		case CELULAR:
			success = toggleLed(GREEN, YELLOW);
			break;
		case TABLET:
			success = toggleLed(RED, BLUE);
			break;
		case NOTEBOOK:
			success = toggleLed(GREEN, RED);
			break;
		default:
			success = false;
	}
	if (departure_place_ == EXAMES && arrival_place_ == SETOR_DE_SAIDA)
	{
		success = toggleLed(GREEN);
	}
	else if (departure_place_ == CENTRO_CIRURGICO && arrival_place_ == SETOR_DE_RECUPERACAO)
	{
		success = toggleLed(RED);
	}
	else if (departure_place_ == SETOR_DE_RECUPERACAO && arrival_place_ == SETOR_DE_SAIDA)
	{
		success = toggleLed(YELLOW);
	}
	else if (departure_place_ == EXAMES && arrival_place_ == CENTRO_CIRURGICO)
	{
		success = toggleLed(BLUE);
	}
	publish();
	return success;
}

bool RobotinoLeds::sinalizeEndOfTask()
{
	ROS_DEBUG("Sinalizing The End!!!");
	ros::Duration d(.5 / frequency_);
	resetLeds();
	publish();
	for (int i = 0; i < 3; i++)
	{
		d.sleep();
		setLeds();
		publish();
		d.sleep();
		resetLeds();
		publish();
	}
	
	return true;
}

bool RobotinoLeds::toggleLed(int led)
{
	bool success = false;
	std::vector<bool> mask(size_, false);
	mask[led] = true;
	for (int i = 0; i < size_; i++)
	{
		digital_msg_.values[i] = digital_msg_.values[i] ? !mask[i] : mask[i]; // xor boolean logic
	}
	return success;
}

bool RobotinoLeds::toggleLed(int led1, int led2)
{
	bool success = false;
	if (!isLighting(led1) && !isLighting(led2))
	{
		toggleLed(led1);
	}
	else
	{
		toggleLed(led1);
		toggleLed(led2);
	}
	return success;
}

bool RobotinoLeds::setLeds()
{
	std::vector<bool> mask(size_, false);
	mask[RED] = true;
	mask[BLUE] = true;
	mask[YELLOW] = true;
	mask[GREEN] = true;
	return setLeds(mask);
}

bool RobotinoLeds::setLeds(std::vector<bool> mask)
{
	if (mask.size() != digital_msg_.values.size())
	{
		ROS_ERROR("mask size must be equals to digital_msg_.values size!!!");
		return false;
	}
	for (int i = 0; i < mask.size(); i++)
	{
		digital_msg_.values[i] = true ? digital_msg_.values[i] || mask[i] : false; // or boolean logic
	}
	return true;		
}

bool RobotinoLeds::resetLeds()
{
	std::vector<bool> mask(size_, false);
	mask[RED] = true;
	mask[BLUE] = true;
	mask[YELLOW] = true;
	mask[GREEN] = true;
	return resetLeds(mask);
}
	
bool RobotinoLeds::resetLeds(std::vector<bool> mask)
{
	if (mask.size() != digital_msg_.values.size())
	{
		ROS_ERROR("mask size must be equals to digital_msg_.values size!!!");
		return false;
	}
	for (int i = 0; i < mask.size(); i++)
	{
		digital_msg_.values[i] = true ? !(!digital_msg_.values[i] || mask[i]) : false; // nonimplication boolean logic
	}
	return true;	
}

bool RobotinoLeds::isLighting(int led)
{	
	return digital_msg_.values[led];
}

std::string RobotinoLeds::placeToString(Place place)
{
	std::string place_name;
	switch (place)
	{
		case ORIGIN:
			place_name = "ORIGIN";
			break;
		case SETOR_DE_CONTROLE:
			place_name = "SETOR_DE_CONTROLE";
			break;
		case EXAMES:
			place_name = "EXAMES";
			break;
		case CENTRO_CIRURGICO:
			place_name = "CENTRO_CIRURGICO";
			break;
		case SETOR_DE_RECUPERACAO:
			place_name = "SETOR_DE_RECUPERACAO";
			break;
		case SETOR_DE_SAIDA:
			place_name = "SETOR_DE_SAIDA";
			break;
	}
	return place_name;
}
