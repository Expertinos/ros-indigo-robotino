/**
 *  StoreModes.h
 *
 *  Version: 1.0.0.0
 *  Created on: 22/10/2015
 *  Modified on: ??/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef STORE_STORE_NUMBER_H_
#define STORE_STORE_NUMBER_H_

#include <string>

namespace store_store_numbers
{

	typedef enum 
	{
		NEAR,
		MIDDLE,
		FAR_AWAY,
		VOLTAR_PRA_CASA,
		LER_POSTE,
		NONE
	} StoreStoreNumberEnum;

	class StoreStoreNumbers
	{

		public: 
	
			static std::string toString(StoreStoreNumberEnum state);
			static StoreStoreNumberEnum newInstance(int mode_code);
			
	};

};

typedef store_store_numbers::StoreStoreNumberEnum StoreStoreNumber;
typedef store_store_numbers::StoreStoreNumbers StoreStoreNumbers;

#endif /* STORE_STORE_NUMBER_H_ */
