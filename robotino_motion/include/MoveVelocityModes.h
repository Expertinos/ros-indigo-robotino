/**
 *  MovevVelocityModes.h
 *
 *  Version: 1.0.0.0
 *  Created on: 20/11/2014
 *  Modified on: 01/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef MOVE_VELOCITY_MODE_H_
#define MOVE_VELOCITY_MODE_H_

#include <string>

namespace velocityModes
{

	typedef enum 
	{
		CONSTANT,
		TRAPEZOIDAL,
		TRAPEZOIDAL_ONLY_ACC,
		TRAPEZOIDAL_ONLY_DEACC,
		TRIANGULAR,
		TRIANGULAR_ONLY_ACC,
		TRIANGULAR_ONLY_DEACC,
		NONE
	} VelocityModeEnum;

	class VelocityModes
	{

		public: 
	
			static std::string toString(VelocityModeEnum state);
			static VelocityModeEnum newInstance(int mode_code);
			
	};

};

typedef velocityModes::VelocityModeEnum VelocityMode;
typedef velocityModes::VelocityModes VelocityModes;

#endif /* MOVE_VELOCITY_MODE_H_ */
