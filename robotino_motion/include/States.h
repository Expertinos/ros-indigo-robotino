/**
 *  StoreStates.h
 *
 *  Version: 1.0.0.0
 *  Created on: 20/11/2014
 *  Modified on: 20/11/2014
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef STORESTATES_H_
#define STORESTATES_H_

namespace StoreStates
{
	typedef enum 
	{
		IDLE, 
		STORING, 
		STOPPING,
		FINISHED
	} SS;
};

typedef StoreStates::SS StoreState;

#endif /* STORESTATES_H_ */
