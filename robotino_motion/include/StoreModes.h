/**
 *  StoreModes.h
 *
 *  Version: 1.0.0.0
 *  Created on: 22/10/2015
 *  Modified on: ??/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef STORE_MODE_H_
#define STORE_MODE_H_

#include <string>

namespace store_modes
{

	typedef enum 
	{
		VISION,
		LASER_SCAN,
		NONE
	} StoreModeEnum;

	class StoreModes
	{

		public: 
	
			static std::string toString(StoreModeEnum state);
			static StoreModeEnum newInstance(int mode_code);
			
	};

};

typedef store_modes::StoreModeEnum StoreMode;
typedef store_modes::StoreModes StoreModes;

#endif /* STORE_MODE_H_ */
