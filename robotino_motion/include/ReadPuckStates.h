/**
 *  ReadPuckStates.h
 *
 *  Version: 1.0.0.0
 *  Created on: 20/11/2014
 *  Modified on: 01/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef READ_PUCK_STATES_H_
#define READ_PUCK_STATES_H_

#include <string>

namespace read_puck_states
{

	typedef enum 
	{
		UNINITIALIZED,
		IDLE, 
		ALIGNING_FRONTAL,
		HEADING_TOWARD_PUCK,
		HEADING_BACKWARD_PUCK,
		GOING_BACK_TO_ORIGIN,
		STOPPING,
		FINISHED,
		LOST
	} ReadPuckStateEnum;

	class ReadPuckStates
	{

		public: 
	
			static std::string toString(ReadPuckStateEnum state);
			
	};

};

typedef read_puck_states::ReadPuckStateEnum ReadPuckState;
typedef read_puck_states::ReadPuckStates ReadPuckStates;

#endif /* READ_PUCK_STATES_H_ */
