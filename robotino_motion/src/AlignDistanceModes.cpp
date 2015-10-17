/**
 *  AlignDistanceModes.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 01/10/2015
 *  Modified on: ??/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "AlignDistanceModes.h"

/**
 *
 */
std::string distance_modes::DistanceModes::toString(distance_modes::DistanceModeEnum mode)
{
	std::string mode_name;
	switch (mode)
	{
		case CLOSE:
			mode_name = "CLOSE";
			break;
		case NORMAL:
			mode_name = "NORMAL";
			break;
		case FAR:
			mode_name = "FAR";
			break;
		default: 
			mode_name = "Nonexistent distance mode";
	}
	return mode_name;
}

/**
 *
 */
distance_modes::DistanceModeEnum distance_modes::DistanceModes::newInstance(int mode_code)
{
	distance_modes::DistanceModeEnum mode;
	switch (mode_code)
	{
		case 0:
			mode = distance_modes::CLOSE;
			break;
		case 1:
			mode = distance_modes::NORMAL;
			break;
		case 2:
			mode = distance_modes::FAR;
			break;
		default:
			mode = distance_modes::NONE;
	}
	return mode;
}
