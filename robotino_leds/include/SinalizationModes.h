/**
 *  SinalizationModes.h
 *
 *  Version: 1.0.0.0
 *  Created on: 09/10/2015
 *  Modified on: 09/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef SINALIZATION_MODES_H_
#define SINALIZATION_MODES_H_

#include <string>
#include <vector>

namespace sinalization_modes
{

	typedef enum 
	{
		BLINK,
		ALTERNATE,
		LIGHT,
		NONE
	} SinalizationModeEnum;

	class SinalizationModes
	{

	public:

		static SinalizationModeEnum toMode(int mode_code);
		static int toCode(SinalizationModeEnum mode);
		static std::string toString(SinalizationModeEnum mode);
		static SinalizationModeEnum getDefault();
		static std::vector<SinalizationModeEnum> getAll();

	};

};

typedef sinalization_modes::SinalizationModeEnum SinalizationMode;
typedef sinalization_modes::SinalizationModes SinalizationModes;

#endif /* SINALIZATION_MODES_H_ */
