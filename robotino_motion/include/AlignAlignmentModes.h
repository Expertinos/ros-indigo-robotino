/**
 *  AlignAlignmentModes.h
 *
 *  Version: 1.0.0.0
 *  Created on: 20/11/2014
 *  Modified on: 01/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef ALIGN_ALIGNMENT_MODE_H_
#define ALIGN_ALIGNMENT_MODE_H_

#include <string>

namespace alignmentModes
{

	typedef enum 
	{
		FRONT,
		RIGHT,
		LEFT,
		BACK,
		FRONT_RIGHT,
		FRONT_LEFT,
		BACK_RIGHT,
		BACK_LEFT,
		NONE
	} AlignmentModeEnum;

	class AlignmentModes
	{

		public: 
	
			static std::string toString(AlignmentModeEnum state);
			static AlignmentModeEnum newInstance(int mode_code);
			
	};

};

typedef alignmentModes::AlignmentModeEnum AlignmentMode;
typedef alignmentModes::AlignmentModes AlignmentModes;

#endif /* ALIGN_ALIGNMENT_MODE_H_ */
