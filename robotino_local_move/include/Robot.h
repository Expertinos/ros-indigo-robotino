/*
 * RobottinoNode.h
 *
 *  Created on: 09.12.2011
 *      Author: indorewala@serviceRobottics.eu
 */

#ifndef Robot_H
#define Robot_H	

class Robot
{
public:
	Robot();
	~Robot();

	float posX;
	float posY;
	float phi;
	int orientation; // 0 -> x+	1 -> y+		2 -> x-		3 -> y-

};

#endif /* Robot_H */
