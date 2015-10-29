/**
 *  ReadOrderStates.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 01/10/2015
 *  Modified on: ??/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ReadOrderStates.h"

/**
 *
 */
std::string read_order_states::ReadOrderStates::toString(read_order_states::ReadOrderStateEnum state)
{
	std::string state_name;
	switch (state)
	{
		case UNINITIALIZED:
			state_name = "Grab Puck Server has not been initialized so far";
			break;
		case IDLE:
			state_name = "Idle";
			break;
		case ALIGNING_LATERAL:
			state_name = "Aligning lateral";
			break;
		case ALIGNING_FRONTAL:
			state_name = "Aligning frontal";
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
			state_name = "List Read";
			break;
		case LOST:
			state_name = "Robot is lost";
			break;
		default: 
			state_name = "Nonexistent state";
	}
	return state_name;
}
