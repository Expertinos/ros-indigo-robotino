/**
 *  StorePuckStates.h
 *
 *  Version: 1.0.0.0
 *  Created on: 20/11/2014
 *  Modified on: 01/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef STORE_PUCK_STATES_H_
#define STORE_PUCK_STATES_H_

#include <string>

namespace store_puck_states
{

	typedef enum 
	{
		UNINITIALIZED,
		IDLE, 
		ALIGNING_FRONTAL,
		ALIGNING_LATERAL,
		HEADING_TOWARD_AREA,
		FINDING_PUCK,
		STORING_PUCK, 
		LEAVING_PUCK,
		GOING_BACK_TO_ORIGIN,
		STOPPING,
		FINISHED,
		LOST
	} StorePuckStateEnum;

	class StorePuckStates
	{

		public: 
	
			static std::string toString(StorePuckStateEnum state);
			
	};

};

typedef store_puck_states::StorePuckStateEnum StorePuckState;
typedef store_puck_states::StorePuckStates StorePuckStates;

#endif /* STORE_PUCK_STATES_H_ */
