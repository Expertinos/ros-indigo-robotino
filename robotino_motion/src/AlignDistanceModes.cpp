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
std::string distanceModes::DistanceModes::toString(distanceModes::DistanceModeEnum mode)
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
distanceModes::DistanceModeEnum distanceModes::DistanceModes::newInstance(int mode_code)
{
	distanceModes::DistanceModeEnum mode;
	switch (mode_code)
	{
		case 0:
			mode = distanceModes::CLOSE;
			break;
		case 1:
			mode = distanceModes::NORMAL;
			break;
		case 2:
			mode = distanceModes::FAR;
			break;
		default:
			mode = distanceModes::NONE;
	}
	return mode;
}
