/*
 * RobotinoLeds.h
 *
 *  Version: 2.0.0.0
 *  Created on: 07/10/2014
 *  Modified on: ??/10/2015
 *      Author: adrianohrl@unifei.edu.br
 */

#ifndef ROBOTINO_LEDS_H
#define ROBOTINO_LEDS_H

#include <ros/ros.h>
#include <vector>
#include "Colors.h"
#include "SinalizationModes.h"

#include "robotino_msgs/DigitalReadings.h"
#include "robotino_leds/Sinalize.h"
#include "robotino_leds/Trigger.h"

/**
 * Yellow LED is located at port DO1 (because YELLOW color code is 1)
 * Blue LED is located at port DO2 (because BLUE color code is 2)
 * Green LED is located at port DO3 (because GREEN color code is 3)
 * Red LED is located at port DO4 (because RED color code is 2)
 */

#define NUMBER_OF_DIGITAL_OUTPUTS 8  

class RobotinoLeds
{

public:

	RobotinoLeds();
	~RobotinoLeds();

	bool spin();

private:

	ros::NodeHandle nh_;
	ros::Publisher digital_pub_;
	ros::ServiceServer sinalize_srv_;
	ros::ServiceServer stop_srv_;

	robotino_msgs::DigitalReadings digital_msg_;

	std::vector<Color> colors_;
	SinalizationMode mode_;
	double rate_;
	ros::Rate loop_rate_;
	int last_toggled_color_index_;

	void publish();	
	bool sinalize(robotino_leds::Sinalize::Request &req, robotino_leds::Sinalize::Response &res);
	bool stop(robotino_leds::Trigger::Request &req, robotino_leds::Trigger::Response &res);

	void sinalize();
	void stop();
	void blink();
	void alternate();

	bool toggleLed(int led);
	bool toggleLed(Color color);
	bool toggleLeds(int led1, int led2);
	bool toggleLeds(Color color1, Color color2);
	bool toggleLeds();
	bool toggleLeds(std::vector<int> leds);
	bool toggleLeds(std::vector<bool> mask);
	bool setLeds();
	bool setLeds(std::vector<bool> mask);
	bool resetLeds();
	bool resetLeds(std::vector<bool> mask);
	bool isLighting(int led);
	bool isLighting(Color color);

};

#endif /* ROBOTINO_LEDS_H */
