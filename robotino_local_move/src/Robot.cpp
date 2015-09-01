#include "Robot.h"

#define positivo true

Robot::Robot()
{
	if( positivo )
		posX = 5.04;
	else
		posX = -5.04;
	posY = 0.20;
	phi = 0;
	orientation = 1;
}

Robot::~Robot()
{
}
