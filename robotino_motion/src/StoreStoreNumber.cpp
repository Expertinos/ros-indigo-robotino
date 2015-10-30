/**
 *  AlignDistanceModes.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 01/10/2015
 *  Modified on: ??/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "StoreStoreNumber.h"

/**
 *
 */
std::string store_store_numbers::StoreStoreNumbers::toString(store_store_numbers::StoreStoreNumberEnum mode)
{
	std::string mode_name;
	switch (mode)
	{
		case NEAR:
			mode_name = "NEAR";
			break;
		case MIDDLE:
			mode_name = "MIDDLE";
			break;
		case FAR_AWAY:
			mode_name = "FAR_AWAY";
			break;
		case VOLTAR_PRA_CASA:
			mode_name = "VOLTAR_PRA_CASA";
			break;
		default: 
			mode_name = "Nonexistent store number";
	}
	return mode_name;
}

/**
 *
 */
store_store_numbers::StoreStoreNumberEnum store_store_numbers::StoreStoreNumbers::newInstance(int mode_code)
{
	store_store_numbers::StoreStoreNumberEnum mode;
	switch (mode_code)
	{
		case 0:
			mode = store_store_numbers::NEAR;
			break;
		case 1:
			mode = store_store_numbers::MIDDLE;
			break;
		case 2:
			mode = store_store_numbers::FAR_AWAY;
			break;
		case 5:
			mode = store_store_numbers::VOLTAR_PRA_CASA;
			break;
		default:
			mode = store_store_numbers::NONE;
	}
	return mode;
}
