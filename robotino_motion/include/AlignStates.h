/**
 *  AlignStates.h
 *
 *  Version: 1.0.0.0
 *  Created on: 20/11/2014
 *  Modified on: 01/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef ALIGN_STATES_H_
#define ALIGN_STATES_H_

#include <string>

namespace alignStates
{

	typedef enum 
	{
		UNINITIALIZED,
		IDLE, 
		ALIGNING, 
		STOPPING,
		FINISHED
	} AlignStateEnum;

	class AlignStates
	{

		public: 
	
			static std::string toString(AlignStateEnum state);
			
	};

};

typedef alignStates::AlignStateEnum AlignState;
typedef alignStates::AlignStates AlignStates;

#endif /* ALIGN_STATES_H_ */
