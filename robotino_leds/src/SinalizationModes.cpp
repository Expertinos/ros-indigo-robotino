/**
 *  SinalizationModes.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 01/10/2015
 *  Modified on: ??/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "SinalizationModes.h"

/**
 *
 */
sinalization_modes::SinalizationModeEnum sinalization_modes::SinalizationModes::toMode(int mode_code)
{
	SinalizationModeEnum mode;
	switch (mode_code)
	{
		case 0:
			mode = BLINK;
			break;
		case 1:
			mode = ALTERNATE;
			break;
		case 2:
			mode = LIGHT;
			break;
		default:
			mode = getDefault();
	}
	return mode;
}

/**
 *
 */
int sinalization_modes::SinalizationModes::toCode(sinalization_modes::SinalizationModeEnum mode)
{
	int mode_code;
	switch (mode)
	{
		case BLINK:
			mode_code = 0;
			break;
		case ALTERNATE:
			mode_code = 1;
			break;
		case LIGHT:
			mode_code = 2;
			break;
		default:			
			mode_code = toCode(getDefault());
	}
	return mode_code;
}

/**
 *
 */
std::string sinalization_modes::SinalizationModes::toString(sinalization_modes::SinalizationModeEnum mode)
{
	std::string mode_name;
	switch (mode)
	{
		case BLINK:
			mode_name = "BLINK";
			break;
		case ALTERNATE:
			mode_name = "ALTERNATE";
			break;
		case LIGHT:
			mode_name = "LIGHT";
			break;
		default:
			mode_name = toString(getDefault());
	}
	return mode_name;
}

/**
 *
 */
sinalization_modes::SinalizationModeEnum sinalization_modes::SinalizationModes::getDefault()
{
	return BLINK; //cuidado com a cor q vc escolher. Pode dar segmentation fault.
}

/**
 *
 */
std::vector<sinalization_modes::SinalizationModeEnum> sinalization_modes::SinalizationModes::getAll()
{
	std::vector<SinalizationModeEnum> modes;
	modes.push_back(BLINK);
	modes.push_back(ALTERNATE);
	modes.push_back(LIGHT);
	return modes;
}
