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

#include <string>

namespace grab_puck_states
{

	typedef enum 
	{
		UNINITIALIZED,
		IDLE, 
		ALIGNING_LATERAL,
		HEADING_TOWARD_PUCK,
		GRABBING_PUCK, 
		ROTATING,
		GOING_BACK_TO_ORIGIN,
		STOPPING,
		FINISHED,
		LOST
	} GrabPuckStateEnum;

	class GrabPuckStates
	{

		public: 
	
			static std::string toString(GrabPuckStateEnum state);
			
	};

};

typedef grab_puck_states::GrabPuckStateEnum GrabPuckState;
typedef grab_puck_states::GrabPuckStates GrabPuckStates;

#endif /* GRAB_PUCK_STATES_H_ */
