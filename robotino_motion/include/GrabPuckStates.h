/**
 *  GrabPuckStates.h
 *
 *  Version: 1.0.0.0
 *  Created on: 20/11/2014
 *  Modified on: 01/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef GRAB_PUCK_STATES_H_
#define GRAB_PUCK_STATES_H_

namespace GrabPuckStates
{
	typedef enum 
	{
		UNINITIALIZED,
		IDLE, 
		FINDING_PUCK,
		GRABING_PUCK, 
		STOPPING,
		FINISHED,
		LOST
	} GPS;
};

typedef GrabPuckStates::GPS GrabPuckState;

#endif /* GRAB_PUCK_STATES_H_ */
