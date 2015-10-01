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
Colors::Colors(Color color) 
{
	setColor(color);
}

/**
 *
 */
void Colors::setColor(int color_code)
{
	code_ = color_code;
	switch (color_code)
	{
		case 0:
			color_ = ORANGE;
			name_ = "ORANGE";
			break;
		case 1:
			color_ = YELLOW;
			name_ = "YELLOW";
			break;
		case 2:
			color_ = BLUE;
			name_ = "BLUE";
			break;
		case 3:
			color_ = GREEN;
			name_ = "GREEN";
			break;
		case 4:
			color_ = RED;
			name_ = "RED";
			break;
		case 5:
			color_ = RED;
			name_ = "BLACK";
			break;
		default:
			code_ = -1;
			name_ = "";
			color_ = NONE;
	}
}

/**
 *
 */
void Colors::setColor(Color color)
{
	color_ = color;
	switch (color)
	{
		case ORANGE:
			code_ = 0;
			name_ = "ORANGE";
			break;
		case YELLOW:
			code_ = 1;
			name_ = "YELLOW";
			break;
		case BLUE:
			code_ = 2;
			name_ = "BLUE";
			break;
		case GREEN:
			code_ = 3;
			name_ = "GREEN";
			break;
		case RED:
			code_ = 4;
			name_ = "RED";
			break;
		case BLACK:
			code_ = 5;
			name_ = "BLACK";
			break;
		default:
			code_ = -1;
			name_ = "";
			color_ = NONE;
	}
}

/**
 *
 */
std::string Colors::toString()
{
	return name_;
}

/**
 *
 */
int Colors::getProductCode()
{
	int product_code;
	switch (color_)
	{
		case ORANGE:
			product_code = 0;
			break;
		case YELLOW:
			product_code = 1;
			break;
		case BLUE:
			product_code = 2;
			break;
		case GREEN:
			product_code = 3;
			break;
		case RED:
			product_code = 4;
			break;
		case BLACK:
			product_code = 5;
			break;
		default:
			product_code = -1;
	}
	return product_code;
}

/**
 *
 */
std::string Colors::getProductString()
{
	std::string product_name;
	switch (color_)
	{
		case ORANGE:
			product_name = "PUCK";
			break;
		case YELLOW:
			product_name = "TV";
			break;
		case BLUE:
			product_name = "DVD";
			break;
		case GREEN:
			product_name = "CELULAR";
			break;
		case RED:
			product_name = "TABLET";
			break;
		case BLACK:
			product_name = "NOTEBOOK";
			break;
		default:
			product_name = "";
	}
	return product_name;
}

/**
 *
 */
Color Colors::convertProductToColor(int product)
{
	Color color;
	switch (product)
	{
		case 0:
			color = ORANGE; // PUCK
			break;
		case 1:
			color = YELLOW; // TV
			break;
		case 2:
			color = BLUE; // DVD
			break;
		case 3:
			color = GREEN; // CELULAR
			break;
		case 4:
			color = RED; // TABLET
			break;
		case 5:
			color = BLACK; // NOTEBOOK
			break;
		case -1: 
			color = NONE;
			break;
		default:
			color = NONE; // PUCK
	}
	return color;
}

/**
 *
 */
int Colors::toProduct(Color color)
{
	int product;
	switch (color)
	{
		case ORANGE: // PUCK
			product = 0;
			break;
		case YELLOW: // TV
			product = 1;
			break;
		case BLUE: // DVD
			product = 2;
			break;
		case GREEN: // CELULAR
			product = 3;
			break;
		case RED: // TABLET
			product = 4;
			break;
		case BLACK: // NOTEBOOK
			product = 5;
			break;
		default: //NONE
			product = -1;
	}
	return product;
}

/**
 *
 */
std::string Colors::toString(Color color)
{
	std::string color_name;
	switch (color)
	{
		case ORANGE: // PUCK
			color_name = "ORANGE";
			break;
		case YELLOW: // TV
			color_name = "YELLOW";
			break;
		case BLUE: // DVD
			color_name = "BLUE";
			break;
		case GREEN: // CELULAR
			color_name = "GREEN";
			break;
		case RED: // TABLET
			color_name = "RED";
			break;
		case BLACK: // NOTEBOOK
			color_name = "BLACK";
			break;
		default: //NONE
			color_name = "";
	}
	return color_name;
}

/**
 *
 */
std::string Colors::convertProductToString(int product)
{
	std::string product_name;
	switch (product)
	{
		case 0:
			product_name = "PUCK";
			break;
		case 1:
			product_name = "TV";
			break;
		case 2:
			product_name = "DVD";
			break;
		case 3:
			product_name = "CELULAR";
			break;
		case 4:
			product_name = "TABLET";
			break;
		case 5:
			product_name = "NOTEBOOK";
			break;
		default:
			product_name = "";
	}
	return product_name;
}

/**
 *
 */
std::string Colors::convertProductToString(Color color)
{
	std::string product_name;
	switch (color)
	{
		case ORANGE: // PUCK
			product_name = "PUCK";
			break;
		case YELLOW: // TV
			product_name = "TV";
			break;
		case BLUE: // DVD
			product_name = "DVD";
			break;
		case GREEN: // CELULAR
			product_name = "CELULAR";
			break;
		case RED: // TABLET
			product_name = "TABLET";
			break;
		case BLACK: // NOTEBOOK
			product_name = "NOTEBOOK";
			break;
		default:
			product_name = "";
	}
	return product_name;
}
