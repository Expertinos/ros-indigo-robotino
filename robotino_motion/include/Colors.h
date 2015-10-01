/**
 *  Colors.h
 *
 *  Version: 1.0.0.0
 *  Created on: 01/10/2015
 *  Modified on: 01/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef COLOR_H_
#define COLOR_H_

#include <string>

typedef enum 
{
	ORANGE, 
	YELLOW, 
	BLUE, 
	GREEN, 
	RED, 
	BLACK, 
	NONE
} Color;

class Colors
{

public:

	Colors(Color color);

	void setColor(int color_code);
	void setColor(Color color);
	std::string toString();
	int getProductCode();
	std::string getProductString();

	static Color convertProductToColor(int product);
	static int toProduct(Color color);
	static std::string toString(Color color);
	static std::string convertProductToString(int product);
	static std::string convertProductToString(Color color);

private:
	
	int code_;
	std::string name_;
	Color color_;

};

#endif /* COLOR_H_ */
