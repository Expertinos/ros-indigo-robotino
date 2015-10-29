/**
 *  ReadOrderStates.h
 *
 *  Version: 1.0.0.0
 *  Created on: 20/11/2014
 *  Modified on: 01/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef READ_ORDER_STATES_H_
#define READ_ORDER_STATES_H_

#include <string>

namespace read_order_states
{

	typedef enum 
	{
		UNINITIALIZED,
		IDLE, 
		ALIGNING_LATERAL,
		ALIGNING_FRONTAL,
		HEADING_TOWARD_PUCK,
		GOING_BACK_TO_ORIGIN,
		STOPPING,
		FINISHED,
		LOST
	} ReadOrderStateEnum;

	class ReadOrderStates
	{

		public: 
	
			static std::string toString(ReadOrderStateEnum state);
			
	};

};

typedef read_order_states::ReadOrderStateEnum ReadOrderState;
typedef read_order_states::ReadOrderStates ReadOrderStates;

#endif /* READ_ORDER_STATES_H_ */
