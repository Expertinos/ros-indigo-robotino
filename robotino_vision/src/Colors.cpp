/**
 *  Colors.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 01/10/2015
 *  Modified on: ??/10/2015
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "Colors.h"

/**
 *
 */
colors::ColorEnum colors::Colors::toColor(int color_code)
{
	ColorEnum color;
	switch (color_code)
	{
		case 0:
			color = ORANGE;
			break;
		case 1:
			color = YELLOW;
			break;
		case 2:
			color = BLUE;
			break;
		case 3:
			color = GREEN;
			break;
		case 4:
			color = RED;
			break;
		case 5:
			color = BLACK;
			break;
		case 6:
			color = PINK;
			break;
		case 7:
			color = PURPLE;
			break;
		default:
			color = NONE;
	}
	return color;
}

/**
 *
 */
int colors::Colors::toCode(colors::ColorEnum color)
{
	int color_code = -1;
	switch (color)
	{
		case ORANGE:
			color_code = 0;
			break;
		case YELLOW:
			color_code = 1;
			break;
		case BLUE:
			color_code = 2;
			break;
		case GREEN:
			color_code = 3;
			break;
		case RED:
			color_code = 4;
			break;
		case BLACK:
			color_code = 5;
			break;
		case PINK:
			color_code = 6;
			break;
		case PURPLE:
			color_code = 7;
			break;
		default:			
			ColorEnum default_color = getDefault();
			color_code = toCode(default_color);
	}
	return color_code;
}

/**
 *
 */
std::string colors::Colors::toString(colors::ColorEnum color)
{
	std::string color_name = "";
	switch (color)
	{
		case ORANGE:
			color_name = "ORANGE";
			break;
		case YELLOW:
			color_name = "YELLOW";
			break;
		case BLUE:
			color_name = "BLUE";
			break;
		case GREEN:
			color_name = "GREEN";
			break;
		case RED:
			color_name = "RED";
			break;
		case BLACK:
			color_name = "BLACK";
			break;
		case PINK:
			color_name = "PINK";
			break;
		case PURPLE:
			color_name = "PURPLE";
			break;
		default:
			ColorEnum default_color = getDefault();
			color_name = toString(default_color);
	}
	return color_name;
}

/**
 *
 */
colors::ColorEnum colors::Colors::getDefault()
{
	return GREEN; //cuidado com a cor q vc escolher. Pode dar segmentation fault.
}

/**
 *
 */
std::vector<colors::ColorEnum> colors::Colors::getAll()
{
	std::vector<ColorEnum> colors;
	colors.push_back(ORANGE);
	colors.push_back(YELLOW);
	colors.push_back(BLUE);
	colors.push_back(GREEN);
	colors.push_back(RED);
	colors.push_back(BLACK);
	colors.push_back(PINK);
	colors.push_back(PURPLE);
	return colors;
}
