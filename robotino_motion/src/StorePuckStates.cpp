/**
 *  StorePuckStates.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 01/10/2015
 *  Modified on: ??/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "StorePuckStates.h"

/**
 *
 */
std::string store_puck_states::StorePuckStates::toString(store_puck_states::StorePuckStateEnum state)
{
	std::string state_name;
	switch (state)
	{
		case UNINITIALIZED:
			state_name = "Store Puck Server has not been initialized so far";
			break;
		case IDLE:
			state_name = "Idle";
			break;
		case ALIGNING_FRONTAL:
			state_name = "Aligning frontal";
			break;
		case ALIGNING_LATERAL:
			state_name = "Aligning lateral";
			break;
		case HEADING_TOWARD_AREA:
			state_name = "Heading toward storage area";
			break;
		case FINDING_PUCK:
			state_name = "Looking for the puck";
			break;
		case STORING_PUCK:
			state_name = "Up to store puck";
			break;
		case LEAVING_PUCK:
			state_name = "Up to leave puck";
			break;
		case GOING_BACK_TO_ORIGIN:
			state_name = "Going back to origin";
			break;
		case STOPPING:
			state_name = "Stopping";
			break;
		case FINISHED:
			state_name = "Puck stored";
			break;
		case LOST:
			state_name = "Kind of lost";
			break;
		default: 
			state_name = "Nonexistent state";
	}
	return state_name;
}
