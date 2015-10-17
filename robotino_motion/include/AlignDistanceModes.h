/**
 *  AlignDistanceModes.h
 *
 *  Version: 1.0.0.0
 *  Created on: 20/11/2014
 *  Modified on: 01/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef ALIGN_DISTANCE_MODE_H_
#define ALIGN_DISTANCE_MODE_H_

#include <string>

namespace distance_modes
{

	typedef enum 
	{
		CLOSE,
		NORMAL,
		FAR,
		NONE
	} DistanceModeEnum;

	class DistanceModes
	{

		public: 
	
			static std::string toString(DistanceModeEnum state);
			static DistanceModeEnum newInstance(int mode_code);
			
	};

};

typedef distance_modes::DistanceModeEnum DistanceMode;
typedef distance_modes::DistanceModes DistanceModes;

#endif /* ALIGN_DISTANCE_MODE_H_ */
