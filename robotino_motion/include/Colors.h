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

namespace colors
{

	typedef enum 
	{
		ORANGE, 
		YELLOW, 
		BLUE, 
		GREEN, 
		RED, 
		BLACK, 
		NONE
	} ColorEnum;

	class Colors
	{

	public:

		static ColorEnum convertProductToColor(int product);
		static int toProduct(ColorEnum color);
		static std::string toString(ColorEnum color);
		static std::string convertProductToString(int product);
		static std::string convertProductToString(ColorEnum color);

	};

};

typedef colors::ColorEnum Color;
typedef colors::Colors Colors;

#endif /* COLOR_H_ */
