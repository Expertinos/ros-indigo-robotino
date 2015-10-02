/**
 *  GrabPuckStates.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 01/10/2015
 *  Modified on: ??/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "GrabPuckStates.h"

/**
 *
 */
std::string grabPuckStates::GrabPuckStates::toString(grabPuckStates::GrabPuckStateEnum state)
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
		case FINDING_PUCK:
			state_name = "Looking for the puck";
			break;
		case GRABBING_PUCK:
			state_name = "Up to grab puck";
			break;
		case STOPPING:
			state_name = "Stopping";
			break;
		case FINISHED:
			state_name = "Puck grabbed";
			break;
		case LOST:
			state_name = "Kind of lost";
			break;
		default: 
			state_name = " Nonexistent state";
	}
	return state_name;
}
