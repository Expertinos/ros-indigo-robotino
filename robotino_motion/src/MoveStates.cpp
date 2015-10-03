/**
 *  MoveStates.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 01/10/2015
 *  Modified on: ??/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "MoveStates.h"

/**
 *
 */
std::string moveStates::MoveStates::toString(moveStates::MoveStateEnum state)
{
	std::string state_name;
	switch (state)
	{
		case UNINITIALIZED:
			state_name = "Move Server has not been initialized so far";
			break;
		case IDLE:
			state_name = "Idle";
			break;
		case MOVING:
			state_name = "Moving";
			break;
		case STOPPING:
			state_name = "Stopping";
			break;
		case FINISHED:
			state_name = "Moveed";
			break;
		default: 
			state_name = "Nonexistent state";
	}
	return state_name;
}
