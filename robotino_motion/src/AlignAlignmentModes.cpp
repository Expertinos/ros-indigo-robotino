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
std::string alignmentModes::AlignmentModes::toString(alignmentModes::AlignmentModeEnum mode)
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
		default: 
			mode_name = "Nonexistent alignment mode";
	}
	return mode_name;
}

/**
 *
 */
alignmentModes::AlignmentModeEnum alignmentModes::AlignmentModes::newInstance(int mode_code)
{
	alignmentModes::AlignmentModeEnum mode;
	switch (mode_code)
	{
		case 0:
			mode = alignmentModes::FRONT;
			break;
		case 1:
			mode = alignmentModes::RIGHT;
			break;
		case 2:
			mode = alignmentModes::LEFT;
			break;
		case 3:
			mode = alignmentModes::BACK;
			break;
		case 4:
			mode = alignmentModes::FRONT_RIGHT;
			break;
		case 5:
			mode = alignmentModes::FRONT_LEFT;
			break;
		case 6:
			mode = alignmentModes::BACK_RIGHT;
			break;
		case 7:
			mode = alignmentModes::BACK_LEFT;
			break;
		default:
			mode = alignmentModes::NONE;
	}
	return mode;
}
