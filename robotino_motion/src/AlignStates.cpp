/**
 *  AlignStates.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 01/10/2015
 *  Modified on: ??/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "AlignStates.h"

/**
 *
 */
std::string alignStates::AlignStates::toString(alignStates::AlignStateEnum state)
{
	std::string state_name;
	switch (state)
	{
		case UNINITIALIZED:
			state_name = "Align Server has not been initialized so far";
			break;
		case IDLE:
			state_name = "Idle";
			break;
		case ALIGNING:
			state_name = "Aligning";
			break;
		case STOPPING:
			state_name = "Stopping";
			break;
		case FINISHED:
			state_name = "Aligned";
			break;
		default: 
			state_name = "Nonexistent state";
	}
	return state_name;
}
