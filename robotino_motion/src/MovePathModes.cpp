/**
 *  MovePathModes.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 01/10/2015
 *  Modified on: ??/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "MovePathModes.h"

/**
 *
 */
std::string pathModes::PathModes::toString(pathModes::PathModeEnum mode)
{
	std::string mode_name;
	switch (mode)
	{
		case TRANSLATIONAL:
			mode_name = "TRANSLATIONAL";
			break;
		case ROTATIONAL:
			mode_name = "ROTATIONAL";
			break;
		case TRANSLATIONAL_AND_ROTATIONAL:
			mode_name = "TRANSLATIONAL and ROTATIONAL";
			break;
		case TANGENT:
			mode_name = "TANGENT";
			break;
		default:
			mode_name = "Nonexistent path mode";
	}
	return mode_name;
}

/**
 *
 */
pathModes::PathModeEnum pathModes::PathModes::newInstance(int mode_code)
{
	pathModes::PathModeEnum mode;
	switch (mode_code)
	{
		case 0:
			mode = pathModes::TRANSLATIONAL;
			break;
		case 1:
			mode = pathModes::ROTATIONAL;
			break;
		case 2:
			mode = pathModes::TRANSLATIONAL_AND_ROTATIONAL;
			break;
		case 3:
			mode = pathModes::TANGENT;
			break;
		default:
			mode = pathModes::NONE;
	}
	return mode;
}
