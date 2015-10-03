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

namespace storePuckStates
{

	typedef enum 
	{
		UNINITIALIZED,
		IDLE, 
		FINDING_PUCK,
		STORING_PUCK, 
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

typedef storePuckStates::StorePuckStateEnum StorePuckState;
typedef storePuckStates::StorePuckStates StorePuckStates;

#endif /* STORE_PUCK_STATES_H_ */
