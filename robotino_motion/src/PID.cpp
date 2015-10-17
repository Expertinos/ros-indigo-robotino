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
	last_error_ = 0;
	sample_time_ = 1 / sample_rate;
	tolerance_ = fabs(tolerance);
	set_point_ = set_point;
	kp_ = kp;
	ki_ = ki;
	kd_ = kd;
}

/**
 *
 */
void PID::compute(double output)
{
	error_ = set_point_ - output;
	i_term_ += sample_time_ * error_;
	if(i_term_ > max_)
	{
		i_term_ = max_;
	}
	else if(i_term_ < min_)
	{
		i_term_ = min_;
	}
	input_ = kp_ * error_ + ki_ * i_term_ + kd_ * (error_ - last_error_) / sample_time_;
	if(input_ > max_)
	{
		input_ = max_;
	}
	else if(input_ < min_)
	{
		input_ = min_;
	}
	last_error_ = error_;
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
	return fabs(error_) < tolerance_;
}

/**
 *
 */
void PID::setKp(double kp)
{
	kp_ = kp;
}

/**
 *
 */
void PID::setKi(double ki)
{
	ki_ = ki;
}

/**
 *
 */
void PID::setKd(double kd)
{
	kd_ = kd;
}
