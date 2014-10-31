/*
 * DigitalOutputArrayROS.cpp
 *
 *  Created on: 07.10.2014
 *      Author: adrianohrl@unifei.edu.br
 */

#include "DigitalOutputArrayROS.h"

DigitalOutputArrayROS::DigitalOutputArrayROS()
{
	digital_sub_ = nh_.subscribe("set_digital_values", 1, &DigitalOutputArrayROS::setDigitalValuesCallback, this);
}

DigitalOutputArrayROS::~DigitalOutputArrayROS()
{
	digital_sub_.shutdown();
}

void DigitalOutputArrayROS::setDigitalValuesCallback(const robotino_msgs::DigitalReadingsConstPtr& msg)
{
	int numValues = msg->values.size();
	if(numValues > 0)
	{
		int values[numValues];
		for (int i = 0; i < numValues; i++)
			values[i] = 1 ? msg->values[i] : 0;
		setValues(values, numValues);
	}
}
