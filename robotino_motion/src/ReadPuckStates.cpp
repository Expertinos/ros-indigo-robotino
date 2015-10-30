/**
 *  ReadPuckStates.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 01/10/2015
 *  Modified on: ??/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ReadPuckStates.h"

/**
 *
 */
std::string read_puck_states::ReadPuckStates::toString(read_puck_states::ReadPuckStateEnum state)
{
	std::string state_name;
	switch (state)
	{
		case UNINITIALIZED:
			state_name = "Read Puck Server has not been initialized so far";
			break;
		case IDLE:
			state_name = "Idle";
			break;
		case HEADING_TOWARD_PUCK:
			state_name = "Heading toward puck";
			break;
		case GOING_BACK_TO_ORIGIN:
			state_name = "Going back to origin";
			break;
		case STOPPING:
			state_name = "Stopping";
			break;
		case FINISHED:
			state_name = "Puck read";
			break;
		case LOST:
			state_name = "Robot is lost";
			break;
		default: 
			state_name = "Nonexistent state";
	}
	return state_name;
}
