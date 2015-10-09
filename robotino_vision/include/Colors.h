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
#include <vector>

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
		PINK,
		PURPLE,
		NONE
	} ColorEnum;

	class Colors
	{

	public:

		static ColorEnum toColor(int color_code);
		static int toCode(ColorEnum color);
		static std::string toString(ColorEnum color);
		static ColorEnum getDefault();
		static std::vector<ColorEnum> getAll();

	};

};

typedef colors::ColorEnum Color;
typedef colors::Colors Colors;

#endif /* COLOR_H_ */
