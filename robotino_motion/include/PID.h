/**
 *  PID.h
 *
 *  Version: 1.0.0.0
 *  Created on: 01/10/2015
 *  Modified on: ??/10/2014
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef PID_H_
#define PID_H_

#include <math.h>

class PID
{

public:

	PID(double set_point, double kp, double ki, double kd, double min, double max, double sample_rate, double tolerance);
	void compute(double output);
	void setSetPoint(double set_point);
	double getInput();
	double getError();
	bool isInSteadyState();

private:

	double kp_, ki_, kd_;
	double i_term_, last_output_;
	double set_point_, error_, input_;
	double min_, max_;
	double sample_time_;
	double tolerance_;

};

#endif /* PID_H_ */
