/**
 *  AlignAlignmentModes.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 01/10/2015
 *  Modified on: ??/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "AlignAlignmentModes.h"

/**
 *
 */
std::string alignment_modes::AlignmentModes::toString(alignment_modes::AlignmentModeEnum mode)
{
	std::string mode_name;
	switch (mode)
	{
		case FRONT:
			mode_name = "FRONT";
			break;
		case RIGHT:
			mode_name = "RIGHT";
			break;
		case LEFT:
			mode_name = "LEFT";
			break;
		case BACK:
			mode_name = "BACK";
			break;
		case FRONT_RIGHT:
			mode_name = "FRONT and RIGHT";
			break;
		case FRONT_LEFT:
			mode_name = "FRONT and LEFT";
			break;
		case BACK_RIGHT:
			mode_name = "BACK and RIGHT";
			break;
		case BACK_LEFT:
			mode_name = "BACK and LEFT";
			break;
		case LASER_FRONT:
			mode_name = "LASER FRONT";
			break;
		case LASER_RIGHT_LEFT:
			mode_name = "LASER RIGHT and LEFT";
			break;
		default: 
			mode_name = "Nonexistent alignment mode";
	}
	return mode_name;
}

/**
 *
 */
alignment_modes::AlignmentModeEnum alignment_modes::AlignmentModes::newInstance(int mode_code)
{
	alignment_modes::AlignmentModeEnum mode;
	switch (mode_code)
	{
		case 0:
			mode = alignment_modes::FRONT;
			break;
		case 1:
			mode = alignment_modes::RIGHT;
			break;
		case 2:
			mode = alignment_modes::LEFT;
			break;
		case 3:
			mode = alignment_modes::BACK;
			break;
		case 4:
			mode = alignment_modes::FRONT_RIGHT;
			break;
		case 5:
			mode = alignment_modes::FRONT_LEFT;
			break;
		case 6:
			mode = alignment_modes::BACK_RIGHT;
			break;
		case 7:
			mode = alignment_modes::BACK_LEFT;
			break;
		case 8:
			mode = alignment_modes::LASER_FRONT;
			break;
		case 9:
			mode = alignment_modes::LASER_RIGHT_LEFT;
			break;
		default:
			mode = alignment_modes::NONE;
	}
	return mode;
}
