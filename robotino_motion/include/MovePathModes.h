/**
 *  MovePathModes.h
 *
 *  Version: 1.0.0.0
 *  Created on: 20/11/2014
 *  Modified on: 01/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef MOVE_PATH_MODE_H_
#define MOVE_PATH_MODE_H_

#include <string>

namespace pathModes
{

	typedef enum 
	{
		TRANSLATIONAL,
		ROTATIONAL,
		TRANSLATIONAL_AND_ROTATIONAL,
		TANGENT,
		NONE
	} PathModeEnum;

	class PathModes
	{

		public: 
	
			static std::string toString(PathModeEnum state);
			static PathModeEnum newInstance(int mode_code);
			
	};

};

typedef pathModes::PathModeEnum PathMode;
typedef pathModes::PathModes PathModes;

#endif /* MOVE_PATH_MODE_H_ */
