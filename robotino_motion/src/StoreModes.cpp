/**
 *  StoreModes.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 22/10/2015
 *  Modified on: ??/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "StoreModes.h"

/**
 *
 */
std::string store_modes::StoreModes::toString(store_modes::StoreModeEnum mode)
{
	std::string mode_name;
	switch (mode)
	{
		case VISION:
			mode_name = "VISION";
			break;
		case LASER_SCAN:
			mode_name = "LASER_SCAN";
			break;
		default: 
			mode_name = "Nonexistent store mode";
	}
	return mode_name;
}

/**
 *
 */
store_modes::StoreModeEnum store_modes::StoreModes::newInstance(int mode_code)
{
	store_modes::StoreModeEnum mode;
	switch (mode_code)
	{
		case 0:
			mode = store_modes::VISION;
			break;
		case 1:
			mode = store_modes::LASER_SCAN;
			break;
		default:
			mode = store_modes::NONE;
	}
	return mode;
}
