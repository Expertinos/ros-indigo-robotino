/**
 *  MoveStates.h
 *
 *  Version: 1.0.0.0
 *  Created on: 20/11/2014
 *  Modified on: 01/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef MOVE_STATES_H_
#define MOVE_STATES_H_

#include <string>

namespace moveStates
{

	typedef enum 
	{
		UNINITIALIZED,
		IDLE, 
		MOVING, 
		STOPPING,
		FINISHED
	} MoveStateEnum;

	class MoveStates
	{

		public: 
	
			static std::string toString(MoveStateEnum state);
			
	};

};

typedef moveStates::MoveStateEnum MoveState;
typedef moveStates::MoveStates MoveStates;

#endif /* MOVE_STATES_H_ */
