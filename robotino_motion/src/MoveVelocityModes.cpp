/**
 *  MoveVelocityModes.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 01/10/2015
 *  Modified on: ??/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "MoveVelocityModes.h"

/**
 *
 */
std::string velocityModes::VelocityModes::toString(velocityModes::VelocityModeEnum mode)
{
	std::string mode_name;
	switch (mode)
	{
		case CONSTANT:
			mode_name = "CONSTANT";
			break;
		case TRAPEZOIDAL:
			mode_name = "TRAPEZOIDAL";
			break;
		case TRAPEZOIDAL_ONLY_ACC:
			mode_name = "TRAPEZOIDAL, ONLY ACCELERATION";
			break;
		case TRAPEZOIDAL_ONLY_DEACC:
			mode_name = "TRAPEZOIDAL, ONLY DEACCELERATION";
			break;
		case TRIANGULAR:
			mode_name = "TRIANGULAR";
			break;
		case TRIANGULAR_ONLY_ACC:
			mode_name = "TRIANGULAR, ONLY ACCELERATION";
			break;
		case TRIANGULAR_ONLY_DEACC:
			mode_name = "TRIANGULAR, ONLY DEACCELERATION";
			break;
		default:
			mode_name = "Nonexistent velocity mode";
	}
	return mode_name;
}

/**
 *
 */
velocityModes::VelocityModeEnum velocityModes::VelocityModes::newInstance(int mode_code)
{
	velocityModes::VelocityModeEnum mode;
	switch (mode_code)
	{
		case 0:
			mode = velocityModes::CONSTANT;
			break;
		case 1:
			mode = velocityModes::TRAPEZOIDAL;
			break;
		case 2:
			mode = velocityModes::TRAPEZOIDAL_ONLY_ACC;
			break;
		case 3:
			mode = velocityModes::TRAPEZOIDAL_ONLY_DEACC;
			break;
		case 4:
			mode = velocityModes::TRIANGULAR;
			break;
		case 5:
			mode = velocityModes::TRIANGULAR_ONLY_ACC;
			break;
		case 6:
			mode = velocityModes::TRIANGULAR_ONLY_DEACC;
			break;
		default:
			mode = velocityModes::NONE;
	}
	return mode;
}
