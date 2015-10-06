/**
 *  PID.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 05/10/2015
 *  Modified on: ??/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "PID.h"

/**
 *
 */
PID::PID(double set_point, double kp, double ki, double kd, double min, double max, double sample_rate, double tolerance)
{
	if(kp < 0 || ki < 0 || kd < 0)
	{
		return;
	}
	if(min <= max)
	{
		min_ = min;
		max_ = max;
	}
	i_term_ = 0;
	last_output_ = 0;
	sample_time_ = 1 / sample_rate;
	tolerance_ = tolerance;
	set_point_ = set_point;
	kp_ = kp;
	ki_ = ki * sample_time_;
	kd_ = kd / sample_time_;
}

/**
 *
 */
void PID::compute(double output)
{
	error_ = set_point_ - output;
	i_term_ += (ki_ * error_);
	if(i_term_ > max_)
	{
		i_term_ = max_;
	}
	else if(i_term_ < min_)
	{
		i_term_ = min_;
	}
	input_ = kp_ * error_ + i_term_ - kd_ * (output - last_output_);
	if(input_ > max_)
	{
		input_ = max_;
	}
	else if(input_ < min_)
	{
		input_ = min_;
	}
	last_output_ = output;
}

/**
 *
 */
void PID::setSetPoint(double set_point)
{
	set_point_ = set_point;
}

/**
 *
 */
double PID::getInput()
{
	return input_;
}

/**
 *
 */
double PID::getError()
{
	return error_;
}

/**
 *
 */
bool PID::isInSteadyState()
{
	return fabs(error_) < tolerance_ * fabs(set_point_);
}
