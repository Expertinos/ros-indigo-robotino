/*
 * RobotinoLeds.cpp
 *
 *  Version: 2.0.0.0
 *  Created on: 07/10/2014
 *  Modified on: ??/10/2015
 *      Author: adrianohrl@unifei.edu.br
 */

#include "RobotinoLeds.h"

/**
 * 
 */
RobotinoLeds::RobotinoLeds():
	loop_rate_(1.0)
{
	digital_pub_ = nh_.advertise<robotino_msgs::DigitalReadings>("set_digital_values", 1, true);
	sinalize_srv_ = nh_.advertiseService("sinalize", &RobotinoLeds::sinalize, this);
	stop_srv_ = nh_.advertiseService("stop_sinalization", &RobotinoLeds::stop, this);
	digital_msg_.values.resize(NUMBER_OF_DIGITAL_OUTPUTS);
	stop();
}

/**
 * 
 */
RobotinoLeds::~RobotinoLeds()
{
	digital_pub_.shutdown();
	sinalize_srv_.shutdown();
	stop_srv_.shutdown();
}

/**
 * 
 */
bool RobotinoLeds::spin()
{
	ROS_INFO("Robotino LED node is up and running!!!");
	while(nh_.ok())
	{
		sinalize();
		ros::spinOnce();
		loop_rate_.sleep();
	}
	return true;
}

/**
 * 
 */
void RobotinoLeds::publish()
{
	digital_msg_.stamp = ros::Time::now();
	digital_pub_.publish(digital_msg_);
}

/**
 * 
 */
bool RobotinoLeds::sinalize(robotino_leds::Sinalize::Request &req, robotino_leds::Sinalize::Response &res)
{
	stop();
	mode_ = SinalizationModes::toMode(req.mode);
	if (mode_ == sinalization_modes::NONE)
	{
		res.success = false;
		res.message = "Invalid sinalization mode";
		ROS_ERROR("Invalid sinalization mode: %d!!!", req.mode);
		return false;
	}	
	colors_.clear();
	last_toggled_color_index_ = 0;
	if (req.colors.empty())
	{
		colors_ = Colors::getAll();
	}
	else
	{
		for (int i = 0; i < req.colors.size(); i++)
		{
			colors_.push_back(Colors::toColor(req.colors[i]));
		}
	}
	if (req.rate <= 0 || mode_ == sinalization_modes::LIGHT)
	{
		mode_ = sinalization_modes::LIGHT;
		setLeds();
		publish();
		res.message = "In " + SinalizationModes::toString(mode_) + " mode!!!";
		res.success = true;
		ROS_DEBUG("%s", res.message.c_str());
		return true;
	}	
	rate_ = req.rate;
	ros::Rate loop_rate(2 * rate_);
	loop_rate_ = loop_rate;
	int times = req.times;
	if (times > 0)
	{
		ROS_DEBUG("In %s mode at %f Hz for %d times", SinalizationModes::toString(mode_).c_str(), rate_, times);
		if (mode_ == sinalization_modes::ALTERNATE)
		{
			times *= colors_.size();
		}
		for (int time = 0; time < times; time++)
		{
			sinalize();
			if (!nh_.ok())
			{
				res.success = false;
				res.message = "Invalid sinalization mode";
				ROS_ERROR("Shuting node down!!!");
				return false;
			}
		}
		res.message = "Sinalized in " + SinalizationModes::toString(mode_) + " mode";
		res.success = true;
		stop();
		return true;		
	} 
	res.message = "Runnig in " + SinalizationModes::toString(mode_) + " mode";
	res.success = true;
	ROS_DEBUG("Runnig in %s mode at %f Hz while any other service call does not happen!!!", SinalizationModes::toString(mode_).c_str(), rate_);
	return true;
}

/**
 * 
 */
bool RobotinoLeds::stop(robotino_leds::Trigger::Request &req, robotino_leds::Trigger::Response &res)
{
	ROS_DEBUG("Stopping Sinalization!!!");
	stop();
	res.success = true;
	res.message = "Sinalization has been aborted!!!";
	if (!res.success)
	{
		res.success = false;
		res.message = "Unable to stop sinalization!!!";
		ROS_ERROR("Unable to stop sinalization!!!");	
		return false;	
	}
	return res.success;
}

/**
 *
 */ 
void RobotinoLeds::sinalize()
{
	switch (mode_)
	{
		case sinalization_modes::BLINK:
			blink();
			break;
		case sinalization_modes::ALTERNATE:
			alternate();
			break;
		case sinalization_modes::LIGHT:
			break;
	}
}

/**
 *
 */
void RobotinoLeds::stop()
{
	nh_.param<double>("rate", rate_, 1.0);
	if (rate_ <= 0)
	{
		rate_ = 1.0;
	}
	ros::Rate loop_rate(2 * rate_);
	loop_rate_ = loop_rate;
	resetLeds();
	publish();
	mode_ = sinalization_modes::NONE;
}

/**
 *
 */
void RobotinoLeds::blink()
{
	ros::Duration d(.5 / rate_);
	setLeds();
	publish();
	d.sleep();
	resetLeds();
	publish();
	d.sleep();
		
}

/**
 *
 */
void RobotinoLeds::alternate()
{
	ros::Duration d(1 / rate_);
	toggleLeds();
	publish();
	d.sleep();
}

/**
 * 
 */
bool RobotinoLeds::toggleLed(int led)
{
	bool success = false;
	std::vector<bool> mask(NUMBER_OF_DIGITAL_OUTPUTS, false);
	mask[led] = true;
	for (int i = 0; i < NUMBER_OF_DIGITAL_OUTPUTS; i++)
	{
		digital_msg_.values[i] = digital_msg_.values[i] ? !mask[i] : mask[i]; // xor boolean logic
	}
	return success;
}

/**
 * 
 */
bool RobotinoLeds::toggleLed(Color color)
{
	return toggleLed(Colors::toCode(color));
}

/**
 * 
 */
bool RobotinoLeds::toggleLeds(int led1, int led2)
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

/**
 * 
 */
bool RobotinoLeds::toggleLeds(Color color1, Color color2)
{
	return toggleLeds(Colors::toCode(color1), Colors::toCode(color2));
}

/**
 * 
 */
bool RobotinoLeds::toggleLeds()
{
	std::vector<int> leds;
	for (int i = 0; i < colors_.size(); i++)
	{
		leds.push_back(Colors::toCode(colors_[i]));
	}
	return toggleLeds(leds);
}

/**
 * 
 */
bool RobotinoLeds::toggleLeds(std::vector<int> leds)
{
	std::vector<bool> mask(NUMBER_OF_DIGITAL_OUTPUTS, false);	
	if (leds.empty())
	{
		ROS_WARN("There are no leds to be toggled!!!");
		return false;
	}
	int led = leds[last_toggled_color_index_];
	if (led < 0 || led > mask.size())
	{
		ROS_ERROR("Invalid mask index: %d!!!", led);
		return false;
	}
	mask[led] = true;
	if (isLighting(led))
	{
		last_toggled_color_index_++;
		if (last_toggled_color_index_ > leds.size() - 1)
		{
			last_toggled_color_index_ = 0;
		}
		led = leds[last_toggled_color_index_];
		if (led < 0 || led > mask.size())
		{
			ROS_ERROR("Invalid mask index: %d!!!", led);
			return false;
		}
		mask[led] = true;
	}	
	return toggleLeds(mask);
}

/**
 * 
 */
bool RobotinoLeds::toggleLeds(std::vector<bool> mask)
{
	if (mask.size() != digital_msg_.values.size())
	{
		ROS_ERROR("mask size must be equals to digital_msg_.values size!!!");
		return false;
	}
	for (int i = 0; i < mask.size(); i++)
	{
		digital_msg_.values[i] = digital_msg_.values[i] ? !mask[i] : mask[i]; // xor boolean logic
	}
	//ROS_DEBUG("Leds have been toggled!!!");
	return true;		
}

/**
 * 
 */
bool RobotinoLeds::setLeds()
{
	std::vector<bool> mask(NUMBER_OF_DIGITAL_OUTPUTS, false);
	for (int i = 0; i < colors_.size(); i++)
	{
		int led = Colors::toCode(colors_[i]);
		mask[led] = true;
	}
	return setLeds(mask);
}

/**
 * 
 */
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
	//ROS_DEBUG("Leds have been setted!!!");
	return true;		
}

/**
 * 
 */
bool RobotinoLeds::resetLeds()
{
	std::vector<bool> mask(NUMBER_OF_DIGITAL_OUTPUTS, false);
	for (int i = 0; i < colors_.size(); i++)
	{
		int led = Colors::toCode(colors_[i]);
		mask[led] = true;
	}
	return resetLeds(mask);
}
	
/**
 * 
 */
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
	//ROS_DEBUG("Leds have been resetted!!!");
	return true;	
}

/**
 * 
 */
bool RobotinoLeds::isLighting(int led)
{	
	return digital_msg_.values[led];
}

/**
 * 
 */
bool RobotinoLeds::isLighting(Color color)
{	
	return isLighting(Colors::toCode(color));
}
