/*
 * DigitalInputArrayROS.cpp
 *
 *  Created on: 07.10.2014
 *      Author: adrianohrl@unifei.edu.br
 */

#include "DigitalInputArrayROS.h"

DigitalInputArrayROS::DigitalInputArrayROS()
{
	digital_pub_ = nh_.advertise<robotino_msgs::DigitalReadings>("digital_readings", 1, true);
}

DigitalInputArrayROS::~DigitalInputArrayROS()
{
	digital_pub_.shutdown();
}

void DigitalInputArrayROS::setTimeStamp(ros::Time stamp)
{
	stamp_ = stamp;
}

void DigitalInputArrayROS::valuesChangedEvent(const int* values, unsigned int size)
{
	digital_msg_.stamp = stamp_;
	digital_msg_.values.resize(size);
	if(size > 0)
	{
		for (int i = 0; i < size; i++)
			digital_msg_.values[i] = true ? values[i] == 1 : false;
		digital_pub_.publish(digital_msg_);
	}

}
