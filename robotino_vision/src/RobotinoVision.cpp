/*
 * RobotinoVision.cpp
 *
 *  Created on: 15/07/2014
 *      Author: adrianohrl@unifei.edu.br
 */

#include "RobotinoVision.h"

/**
 *
 */
RobotinoVision::RobotinoVision():
	it_(nh_)
{
	readParameters();

	find_objects_srv_ = nh_.advertiseService("find_objects", &RobotinoVision::findObjects, this);
	get_list_srv_ = nh_.advertiseService("get_objects_list", &RobotinoVision::getList, this);
	contain_in_list_srv_ = nh_.advertiseService("contain_in_list", &RobotinoVision::containInList, this);
	image_sub_ = it_.subscribe("image_raw", 1, &RobotinoVision::imageCallback, this);
	save_srv_ = nh_.advertiseService("save_image", &RobotinoVision::saveImage, this);
	set_calibration_srv_ = nh_.advertiseService("set_calibration", &RobotinoVision::setCalibration, this);

	imgRGB_ = cv::Mat(width_, height_, CV_8UC3, cv::Scalar::all(0));

	setColor();	
	contours_window_name_ = CONTOURS_WINDOW + ": " + Colors::toString(color_);		

	calibration_ = false;
	close_aux_ = 5;
	open_aux_ = 2;
	dilate_aux_ = 10;
	max_area_ = 10;

	verify_markers_ = true;
	specific_number_of_markers_ = -1;
	
	setImagesWindows();
}

/**
 *
 */
RobotinoVision::~RobotinoVision()
{
	imgRGB_.release();
	find_objects_srv_.shutdown();
	image_sub_.shutdown();
	save_srv_.shutdown();
	set_calibration_srv_.shutdown();
	cv::destroyAllWindows();
}

/**
 *
 */
bool RobotinoVision::spin()
{
	ROS_INFO("Robotino Vision Node up and running!!!");
	ros::Rate loop_rate(30);
	while(nh_.ok())
	{
		std::vector<cv::Point2f> mass_center = processColor();
		std::vector<cv::Point2f> positions = getPositions(mass_center);
		for (int i = 0; i < positions.size(); i++)
		{		
			ROS_DEBUG("(distance=%f,direction=%f)", positions[i].x, positions[i].y);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return true;
}

/**
 *
 */
bool RobotinoVision::findObjects(robotino_vision::FindObjects::Request &req, robotino_vision::FindObjects::Response &res)
{
	Color color = Colors::toColor(req.color);
	setColor(color);
	verify_markers_ = req.verify_markers;
	specific_number_of_markers_ = req.specific_number_of_markers;
	std::vector<cv::Point2f> mass_center = processColor();
	std::vector<cv::Point2f> positions = getPositions(mass_center);
	int number_of_objects = positions.size();
	for (int k = 0; k < number_of_objects; k++)
	{
		res.distances.push_back(positions[k].x);
		res.directions.push_back(positions[k].y);
		res.number_of_markers.push_back(number_of_markers_[k]);
	}
	return true;
}

/**
 *
 */
bool RobotinoVision::getList(robotino_vision::GetObjectsList::Request &req, robotino_vision::GetObjectsList::Response &res)
{
	res.succeed = false;
	std::vector<Color> orderedColors = getObjectsInOrder();
	for (int i = 0; i < orderedColors.size(); i++)
	{
		Color color = orderedColors.at(i);
		res.objects.push_back(Colors::toCode(color));
	}
	if (!res.objects.empty())
	{
		res.succeed = true;
	}
	setColor();
	return true;
}

/**
 *
 */
bool RobotinoVision::containInList(robotino_vision::ContainInList::Request &req, robotino_vision::ContainInList::Response &res)
{
	if (req.objects.empty())
	{
		return false;
	}
	bool contain = false;
	Color closestObjectColor = colors::NONE;
	float closestObjectDistance = 100000;
	for (int i = 0; i < req.objects.size(); i++) 
	{
		Color objectColor = Colors::toColor(req.objects.at(i));
		float objectDistance = getClosestObjectDistance(objectColor);
		if (objectDistance > 0)
		{
			contain = true;
			if (objectDistance < closestObjectDistance || closestObjectColor == colors::NONE) 
			{
				closestObjectDistance = objectDistance;
				closestObjectColor = objectColor;
			}
		}		
	}
	res.object = -1;
	if (closestObjectColor != colors::NONE)
	{
		res.object = Colors::toCode(closestObjectColor);
	}
	res.contain = contain;
	return true;
}

/**
 *
 */
void RobotinoVision::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	} 
	imgRGB_ = cv_ptr->image;
}

/**
 *
 */
bool RobotinoVision::readImage(std::string image_name)
{
	cv::Mat imgBGR = cv::imread(image_name.c_str(), 1);
	if (!imgBGR.data)
	{
		imgRGB_ = cv::Mat(width_, height_, CV_8UC3, cv::Scalar::all(0));
	}
	cv::cvtColor(imgBGR, imgRGB_, CV_BGR2RGB);
	return true;
}

/**
 * returns the total number of objects considering all colors
 */
int RobotinoVision::getNumberOfObjects()
{
	int numberOfObjects = 0;
	std::vector<Color> colors = Colors::getAll();
	for (int i = 0; i < colors.size(); i++)
	{	
		Color color = colors.at(i);
		numberOfObjects += getNumberOfObjects(color);
	}
	return numberOfObjects;
}

/**
 * returns the total number of objects considering a specific
 */
int RobotinoVision::getNumberOfObjects(Color color)
{
	setColor(color);
	std::vector<cv::Point2f> points = processColor();
	int numberOfObjects = points.size();
	return numberOfObjects;
}

/**
 * This method get objetcs of all colors and order them from left to right
 */
std::vector<Color> RobotinoVision::getObjectsInOrder()
{
	return getObjectsInOrder(true);
}

/**
 * This method get objetcs of all colors and order them from left to right, if the boolean input parameter, from_left_to_right, is true
 * (otherwise, this method order them from right to left
 */
std::vector<Color> RobotinoVision::getObjectsInOrder(bool from_left_to_right)
{
	std::vector<Object> objects;
	std::vector<Color> colors = Colors::getAll();
	for (int i = 0; i < colors.size(); i++)
	{	
		Color color = colors.at(i);
		std::vector<cv::Point2f> objects_mass_center = processColor(color);
		std::vector<cv::Point2f> objects_position = getPositions(objects_mass_center);
		for (int j = 0; j < objects_position.size(); j++)
		{
			cv::Point2f object_position = objects_position.at(j);
			Object object;
			object.x = object_position.x;
			object.y = object_position.y;
			object.color = color;
			objects.push_back(object);
		}
	}
	return orderObjects(objects, from_left_to_right);
}

/**
 * order using bubble sort method
 */
std::vector<Color> RobotinoVision::orderObjects(std::vector<Object> cluttered_objects, bool from_left_to_right)
{
	for(int i = cluttered_objects.size() - 1; i > 0; i--) 
	{  
		for(int j = 0; j < i ; j++) 
		{
			Object obj1 = cluttered_objects.at(j);
			Object obj2 = cluttered_objects.at(j + 1);
			if(obj1.y > obj2.y) 
			{
				cluttered_objects[j] = obj2;
				cluttered_objects[j + 1] = obj1;
			}
		}
	}	
	std::vector<Color> ordered_objects;
	if (from_left_to_right)
	{
		for (int i = 0; i < cluttered_objects.size(); i++)
		{
			ordered_objects.push_back(cluttered_objects.at(i).color);
		}
	}
	else
	{
		for (int i = cluttered_objects.size() - 1; i >= 0; i++)
		{
			ordered_objects.push_back(cluttered_objects.at(i).color);
		}
	}
	return ordered_objects;
}

/**
 *
 */
float RobotinoVision::getClosestObjectDistance(Color color)
{
	setColor(color);
	std::vector<cv::Point2f> points = processColor();
	if (points.empty())
	{
		return -1;		
	}
	std::vector<cv::Point2f> positions = getPositions(points);
	float closestDistance = 100000;
	for (int i = 0; i < positions.size(); i++)
	{
		float distance = positions[i].x;
		if (distance < closestDistance)
		{
			closestDistance = distance;
		}
	}
	return closestDistance;
}

/**
 *
 */
std::vector<cv::Point2f> RobotinoVision::processColor(Color color) 
{
	setColor(color);
	return processColor();
}

/**
 *
 */
std::vector<cv::Point2f> RobotinoVision::processColor()
{
	std::vector<cv::Point2f> points;
	if (color_ != colors::YELLOW && color_ != colors::BLUE && color_ != colors::GREEN && color_ != colors::RED)
	{
		return points;
	}
	ROS_DEBUG("Getting Pucks Mask!!!");
	cv::Mat pucks_mask = getPucksMask();
	ROS_DEBUG("Getting Color Mask!!!");
	cv::Mat color_mask = getColorMask();
	ROS_DEBUG("Getting Final Puck Mask!!!");
	cv::Mat final_mask = getFinalMask(pucks_mask, color_mask);
	ROS_DEBUG("Getting Black Mask!!!");
	cv::Mat black_mask = getBlackMask();
	ROS_DEBUG("Getting Puck Markers!!!");
	cv::Mat puck_markers = getPuckMarkers(black_mask, pucks_mask);
	ROS_DEBUG("Getting Puck without Markers!!!");
	cv::Mat puck_without_markers = getPuckWithoutMarkers(final_mask, puck_markers);
	ROS_DEBUG("Getting Contours based on Final Mask!!!");
	
	if (calibration_)
	{
		std::string contours_window_name = CONTOURS_WINDOW + ": " + Colors::toString(color_);
		if (contours_window_name != contours_window_name_) 
		{
			contours_window_name_ = contours_window_name;
			without_markers_window_name_ = Colors::toString(color_) + " " + PUCK_WITHOUT_MARKERS_WINDOW;
			setImagesWindows();
		}	
		cv::createTrackbar("Threshold: ", BLACK_MASK_WINDOW, &color_params_.thresh_0, 255);
		cv::createTrackbar("Open: ", BLACK_MASK_WINDOW, &open_aux_, 20);
		//cv::imshow(BLACK_MASK_WINDOW, black_mask);

		cv::createTrackbar("Threshold: ", PUCKS_MASK_WINDOW, &color_params_.thresh_1, 255);
		cv::createTrackbar("Close: ", PUCKS_MASK_WINDOW, &color_params_.close_1, 20);
		cv::createTrackbar("Open: ", PUCKS_MASK_WINDOW, &color_params_.open_1, 20);
		//cv::imshow(PUCKS_MASK_WINDOW, pucks_mask);
	
		cv::createTrackbar("Initial range value: ", COLOR_MASK_WINDOW, &color_params_.initial_range_value, 255);
		cv::createTrackbar("Range width: ", COLOR_MASK_WINDOW, &color_params_.range_width, 255);
		cv::createTrackbar("Close: ", COLOR_MASK_WINDOW, &close_aux_, 20);
		//cv::imshow(COLOR_MASK_WINDOW, color_mask);
	
		cv::createTrackbar("Open(before): ", FINAL_MASK_WINDOW, &color_params_.open_2, 20);
		cv::createTrackbar("Close: ", FINAL_MASK_WINDOW, &color_params_.close_2, 20);
		cv::createTrackbar("Open(after): ", FINAL_MASK_WINDOW, &color_params_.open_3, 20);
		cv::createTrackbar("Dilate: ", FINAL_MASK_WINDOW, &dilate_aux_, 20);
		//cv::imshow(FINAL_MASK_WINDOW, final_mask);
	
		cv::createTrackbar("Max Marker Area Size: ", contours_window_name_, &max_area_, MIN_AREA);
		cv::imshow(PUCK_MARKERS_WINDOW, puck_markers);

		cv::createTrackbar("Min Puck Area Size: ", contours_window_name_, &color_params_.min_area, 10 * MIN_AREA);
	
		showImageBGRwithMask(final_mask);
	}

	black_mask.release();
	pucks_mask.release();
	color_mask.release();
	puck_markers.release();

	points = getContours(puck_without_markers);///////////////////////final_mask);

	final_mask.release();
	
	cv::waitKey(30);

	return points;
}

/**
 *
 */
cv::Mat RobotinoVision::getBlackMask()
{
	cv::Mat black_mask;

	// convertendo de RGB para HSV
	cv::Mat imgHSV;
	cv::cvtColor(imgRGB_, imgHSV, CV_RGB2HSV);

	// separando a HSV 
	cv::Mat splitted[3];
	cv::split(imgHSV, splitted);
	black_mask = splitted[2];

	// fazendo threshold da imagem V
	cv::threshold(black_mask, black_mask, color_params_.thresh_0, 255, cv::THRESH_BINARY);

	////////////////////
	cv::Mat element;
	// fechando buracos
	/*element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(2 * close_aux_ + 1, 2 * close_aux_ + 1), cv::Point(close_aux_, close_aux_));
	cv::morphologyEx(black_mask, black_mask, 3, element);
*/
	// filtro de partícula pequenas
	element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(2 * open_aux_ + 1, 2 * open_aux_ + 1), cv::Point(open_aux_, open_aux_));
	cv::morphologyEx(black_mask, black_mask, 2, element);

	// fazendo dilatação na imagem acima
	element = getStructuringElement(cv::MORPH_RECT, cv::Size(2 * dilate_aux_ + 1, 2 * dilate_aux_ + 1), cv::Point(dilate_aux_, dilate_aux_));
	/////////////////////cv::dilate(black_mask, black_mask, element);
	
	if (calibration_)
	{
		cv::imshow(BLACK_MASK_WINDOW, black_mask);
	}

	imgHSV.release();
	splitted[0].release();
	splitted[1].release();
	splitted[2].release();
	element.release();

	return black_mask;
}

/**
 *
 */
cv::Mat RobotinoVision::getPucksMask()
{
	cv::Mat pucks_mask;
	
	// convertendo de RGB para HSV
	cv::Mat imgHSV;
	cv::cvtColor(imgRGB_, imgHSV, CV_RGB2HSV);

	// separando a HSV 
	cv::Mat splitted[3];
	cv::split(imgHSV, splitted);
	pucks_mask = splitted[1];

	// fazendo threshold da imagem S
	cv::threshold(pucks_mask, pucks_mask, color_params_.thresh_1, 255, cv::THRESH_BINARY);

	// fechando buracos
	cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(2 * color_params_.close_1 + 1, 2 * color_params_.close_1 + 1), cv::Point(color_params_.close_1, color_params_.close_1));
	cv::morphologyEx(pucks_mask, pucks_mask, 3, element);

	// filtro de partícula pequenas
	element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(2 * color_params_.open_1 + 1, 2 * color_params_.open_1 + 1), cv::Point(color_params_.open_1, color_params_.open_1));
	cv::morphologyEx(pucks_mask, pucks_mask, 2, element);

	if (calibration_)
	{
		cv::imshow(PUCKS_MASK_WINDOW, pucks_mask);
	}

	imgHSV.release();
	splitted[0].release();
	splitted[1].release();
	splitted[2].release();
	element.release();

	return pucks_mask;
}

/**
 *
 */
cv::Mat RobotinoVision::getColorMask()
{
	cv::Mat color_mask;

	// convertendo de RGB para HLS
	cv::Mat imgHLS;
	cv::cvtColor(imgRGB_, imgHLS, CV_RGB2HLS);

	// separando a HLS 
	cv::Mat splitted[3];
	cv::split(imgHLS, splitted);
	color_mask = 1.41666 * splitted[0];

	// rodando a roleta da imagem H
	cv::Mat unsaturated = color_mask - color_params_.initial_range_value;
	cv::Mat saturated = color_mask + (255 - color_params_.initial_range_value); 
	cv::Mat aux;
	cv::threshold(saturated, aux, 254, 255, cv::THRESH_BINARY);
	aux = 255 - aux; 
	cv::bitwise_and(saturated, aux, saturated);
	color_mask = unsaturated + saturated;

	// define o intervalo da cor
	cv::threshold(color_mask, color_mask, color_params_.range_width, 255, cv::THRESH_BINARY);
	color_mask = 255 - color_mask;
////////////////////////////
	// fechando buracos
	cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(2 * close_aux_ + 1, 2 * close_aux_ + 1), cv::Point(close_aux_, close_aux_));
	cv::morphologyEx(color_mask, color_mask, 3, element);

	// filtro de partícula pequenas
	element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(2 * open_aux_ + 1, 2 * open_aux_ + 1), cv::Point(open_aux_, open_aux_));
	cv::morphologyEx(color_mask, color_mask, 2, element);
//////////////////////
	if (calibration_)
	{
		cv::imshow(COLOR_MASK_WINDOW, color_mask);
	}

	imgHLS.release();
	splitted[0].release();
	splitted[1].release();
	splitted[2].release();
	unsaturated.release();
	saturated.release();
	aux.release();
	element.release();

	return color_mask;
}

/**
 *
 */
cv::Mat RobotinoVision::getFinalMask(cv::Mat &black_mask, cv::Mat &pucks_mask, cv::Mat &color_mask)
{
	// juntando todas as máscaras
	cv::Mat final_mask;
	cv::bitwise_and(pucks_mask, black_mask, final_mask);
	cv::bitwise_and(final_mask, color_mask, final_mask);

	// removendo particulas pequenas
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * color_params_.open_2 + 1, 2 * color_params_.open_2 + 1), cv::Point(color_params_.open_2, color_params_.open_2));
	cv::morphologyEx(final_mask, final_mask, 2, element);
	
	// fechando buracos
	element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * color_params_.close_2 + 1, 2 * color_params_.close_2 + 1), cv::Point(color_params_.close_2, color_params_.close_2));
	morphologyEx(final_mask, final_mask, 3, element);

	// removendo particulas pequenas
	element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * color_params_.open_3 + 1, 2 * color_params_.open_3 + 1), cv::Point(color_params_.open_3, color_params_.open_3));
	cv::morphologyEx(final_mask, final_mask, 2, element);

	element.release();

	return final_mask;
}

/**
 *
 */
cv::Mat RobotinoVision::getFinalMask(cv::Mat &pucks_mask, cv::Mat &color_mask)
{
	// juntando todas as máscaras
	cv::Mat final_mask;
	cv::bitwise_and(pucks_mask, color_mask, final_mask);

	// removendo particulas pequenas
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * color_params_.open_2 + 1, 2 * color_params_.open_2 + 1), cv::Point(color_params_.open_2, color_params_.open_2));
	cv::morphologyEx(final_mask, final_mask, 2, element);
	
	// fechando buracos
	element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * color_params_.close_2 + 1, 2 * color_params_.close_2 + 1), cv::Point(color_params_.close_2, color_params_.close_2));
	morphologyEx(final_mask, final_mask, 3, element);

	// removendo particulas pequenas
	element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * color_params_.open_3 + 1, 2 * color_params_.open_3 + 1), cv::Point(color_params_.open_3, color_params_.open_3));
	cv::morphologyEx(final_mask, final_mask, 2, element);

	// fazendo dilatação na imagem acima
	element = getStructuringElement(cv::MORPH_RECT, cv::Size(2 * dilate_aux_ + 1, 2 * dilate_aux_ + 1), cv::Point(dilate_aux_, dilate_aux_));
	cv::dilate(final_mask, final_mask, element);

	if (calibration_)
	{
		cv::imshow(FINAL_MASK_WINDOW, final_mask);
	}

	element.release();

	return final_mask;
}

/**
 *
 */
cv::Mat RobotinoVision::getPuckMarkers(cv::Mat &black_mask, cv::Mat &pucks_mask)
{
	// juntando todas as máscaras
	cv::Mat puck_markers;
	cv::bitwise_and(pucks_mask, black_mask, puck_markers);
	return puck_markers;
}

/**
 *
 */
cv::Mat RobotinoVision::getPuckWithoutMarkers(cv::Mat &final_mask, cv::Mat &puck_markers)
{
	// juntando todas as máscaras
	cv::Mat puck_without_markers = final_mask - puck_markers;
	
	if (calibration_)
	{
		cv::imshow(without_markers_window_name_.c_str(), puck_without_markers);
	}

	return puck_without_markers;
}

/**
 *
 */
void RobotinoVision::showImageBGRwithMask(cv::Mat &mask)
{
	cv::Mat imgBGR;
	cv::cvtColor(imgRGB_, imgBGR, CV_RGB2BGR);
	cv::imshow(BGR_WINDOW, imgBGR);

	cv::Mat splitted[3];
	cv::split(imgBGR, splitted);
	
	std::vector<cv::Mat> channels;
	for (int i = 0; i < 3; i++)
	{
		cv::bitwise_and(splitted[i], mask, splitted[i]);
		channels.push_back(splitted[i]);
	}
	cv::merge(channels, imgBGR);

	splitted[0].release();
	splitted[1].release();
	splitted[2].release();
	channels[0].release();
	channels[1].release();
	channels[2].release();
	imgBGR.release();
}

/**
 *
 */
std::vector<cv::Point2f> RobotinoVision::getContours(cv::Mat &input)
{
	std::vector<std::vector<cv::Point> > initial_contours;
	std::vector<cv::Vec4i> hierarchy;

	/// Find contours
	cv::findContours(input, initial_contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	
	std::vector<std::vector<cv::Point> > contours;
	number_of_markers_.clear();
	int parent = 0;
	for (int i = 0; i < initial_contours.size(); i++)
	{
		float area = cv::contourArea(initial_contours[i]);
		//ROS_WARN("Contour %d: (next: %d|previous: %d|1st_child: %d|parent: %d): %f", i, hierarchy[i][0], hierarchy[i][1], hierarchy[i][2], hierarchy[i][3], area);
		if (!verify_markers_ && area > color_params_.min_area)
		{
			contours.push_back(initial_contours[i]);
		} 
		else if (verify_markers_ && area > 25)
		{
			bool has_parent = hierarchy[i][3] != -1;
			if (!has_parent)
			{
				contours.push_back(initial_contours[i]);
				number_of_markers_.push_back(0);
				int child = hierarchy[i][2];
				while (child != -1)
				{	
					area = cv::contourArea(initial_contours[child]);
					if (area > 25)
					{
						number_of_markers_[parent]++;
					}
					child = hierarchy[child][0];
				}
				parent++;
			}
		}
	}
	std::vector<cv::Point2f> mass_centers;
	if (contours.empty())
	{
		return mass_centers;
	}
	if (verify_markers_ && specific_number_of_markers_ > 0)
	{
		for (int i = contours.size() - 1; i >= 0; i--)
		{
			if (number_of_markers_[i] != specific_number_of_markers_)
			{
				number_of_markers_.erase(number_of_markers_.begin() + i);
				contours.erase(contours.begin() + i);
			}
		}
	}

	/// Get the moments
	std::vector<cv::Moments> mu(contours.size());
	for(int i = 0; i < contours.size(); i++)
	{
		if (verify_markers_)
		{
			ROS_DEBUG("Contour %d has %d markers", i, number_of_markers_[i]);
		}
		mu[i] = moments(contours[i], false); 
	}
	///  Get the mass centers:
	mass_centers.resize(contours.size());
	for(int i = 0; i < contours.size(); i++)
	{
		mass_centers[i] = cv::Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00); 
	}
		
	cv::RNG rng(12345);
	/// Draw contours
	cv::Mat drawing = cv::Mat::zeros(input.size(), CV_8UC3);
	for(int i = 0; i< contours.size(); i++)
	{
		cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
		cv::drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
		cv::circle(drawing, mass_centers[i], 4, color, -1, 8, 0);
		ROS_DEBUG("P%d = (xc, yc) = (%f, %f)", i, mass_centers[i].x, mass_centers[i].y);
	}

	/// Show in a window
	std::string contours_window_name = CONTOURS_WINDOW + ": " + Colors::toString(color_);
	if (contours_window_name != contours_window_name_) 
	{
		cv::destroyWindow(contours_window_name_);
		contours_window_name_ = contours_window_name;
		cv::namedWindow(contours_window_name_);
	}
	cv::imshow(contours_window_name_.c_str(), drawing);///////////////////////////////

	int numberOfObjects = mass_centers.size();
	if (numberOfObjects > MAX_NUMBER_OF_PUCKS) 
	{
		ROS_ERROR("You must recalibrate the %s color parameters, there are too much noise!!!", Colors::toString(color_).c_str());
	}
	return mass_centers;
}

/**
 *
 */
std::vector<cv::Point2f> RobotinoVision::getPositions(std::vector<cv::Point2f> mass_center)
{
	double alpha = atan(camera_close_distance_ / camera_height_);
	double beta = atan(camera_far_distance_ / camera_height_);
	std::vector<cv::Point2f> positions(mass_center.size());

	for (int k = 0; k < positions.size(); k++)
	{
		double i = mass_center[k].x;
		double j = mass_center[k].y;
		double theta = (j * (alpha - beta) / height_) + beta;
		double distance = camera_height_ * tan(theta);
		double gama = atan(.5 * camera_depth_width_ / camera_close_distance_);
		double direction =  gama * (i - width_ / 2) / (width_ / 2);
		positions[k] = cv::Point2f(distance, direction);
		if (calibration_)
		{
			ROS_DEBUG("(%d): Distance = %f, Direction = %f", k, distance, direction * 180 / PI);
		}
	}
	return positions;
}

/**
 * Sets to default color (defined in Colors.cpp)
 */
void RobotinoVision::setColor()
{
	Color color = Colors::getDefault();
	setColor(color);
}

/**
 *
 */
void RobotinoVision::setColor(Color color)
{
	color_ = color;
	setColorParameters();
}

/**
 *
 */
void RobotinoVision::setColorParameters()
{
	switch (color_)
	{
		/*case colors::ORANGE:
			color_params_ = orange_params_;
			break;*/
		case colors::YELLOW:
			color_params_ = yellow_params_;
			break;
		case colors::BLUE:
			color_params_ = blue_params_;
			break;
		case colors::GREEN:
			color_params_ = green_params_;
			break;
		case colors::RED:
			color_params_ = red_params_;
			break;
		case colors::ORANGE:
		case colors::BLACK:
		case colors::PINK:
		case colors::PURPLE:
			//ROS_ERROR("%s color is not working yet!!!", Colors::toString(color_).c_str());
			break;
		default:
			setColor();
	}
}

/**
 *
 */
bool RobotinoVision::saveImage(robotino_vision::SaveImage::Request &req, robotino_vision::SaveImage::Response &res)
{
	cv::imwrite(req.image_name.c_str(), imgRGB_);
	return true;
}

/**
 *
 */
void RobotinoVision::setImagesWindows() 
{	
	cv::destroyAllWindows();
	cv::namedWindow(contours_window_name_.c_str());
	cv::namedWindow(without_markers_window_name_.c_str());
	if (!calibration_) 
	{
		return;
	}
	cv::namedWindow(BLACK_MASK_WINDOW);
	cv::namedWindow(PUCKS_MASK_WINDOW);
	cv::namedWindow(COLOR_MASK_WINDOW);
	cv::namedWindow(FINAL_MASK_WINDOW);
	cv::namedWindow(BGR_WINDOW);
	cv::namedWindow(PUCK_MARKERS_WINDOW);
	cv::namedWindow(contours_window_name_.c_str());
	cv::moveWindow(BLACK_MASK_WINDOW, 1 * width_, 4 * height_);
	cv::moveWindow(PUCKS_MASK_WINDOW, 2 * width_, 4 * height_);
	cv::moveWindow(COLOR_MASK_WINDOW, 3 * width_, 4 * height_);
	cv::moveWindow(FINAL_MASK_WINDOW, 1 * width_, 1 * height_);
	cv::moveWindow(BGR_WINDOW, 2 * width_, 1 * height_);
	cv::moveWindow(contours_window_name_.c_str(), 3 * width_, 1 * height_);
}

/**
 *
 */
bool RobotinoVision::setCalibration(robotino_vision::SetCalibration::Request &req, robotino_vision::SetCalibration::Response &res)
{
	calibration_ = req.calibration;
	setImagesWindows();
	return true;
}

/**
 *
 */
void RobotinoVision::readParameters()
{	
	//which were defined in config/camera_params.yaml file
	nh_.param<double>("/robotino_vision_node/camera/height", camera_height_, 28.0); // in centimeters
	nh_.param<double>("/robotino_vision_node/camera/close_distance", camera_close_distance_, 20.0); // in centimeters
	nh_.param<double>("/robotino_vision_node/camera/far_distance", camera_far_distance_, 62.0); // in centimeters
	nh_.param<double>("/robotino_vision_node/camera/depth_width", camera_depth_width_, 50.0); // in centimeters

	ROS_DEBUG("************* Camera Position ************* ");
	ROS_DEBUG("~/camera/height: %f", camera_height_); 
	ROS_DEBUG("~/camera/close_distance: %f", camera_close_distance_);
	ROS_DEBUG("~/camera/far_distance: %f", camera_far_distance_);
	ROS_DEBUG("~/camera/depth_width: %f", camera_depth_width_);

	nh_.param<int>("/robotino_vision_node/image/height", height_, 240); // in pixels
	nh_.param<int>("/robotino_vision_node/image/width", width_, 320); // in pixels

	ROS_DEBUG("************ Image Size ***************** ");
	ROS_DEBUG("~/image/height: %d", height_);
	ROS_DEBUG("~/image/width: %d", width_);

	//which were defined in config/color_params.yaml file
	nh_.param<int>("/robotino_vision_node/color/orange/thresh_0", orange_params_.thresh_0, 137);
	nh_.param<int>("/robotino_vision_node/color/orange/erosion_0", orange_params_.erosion_0, 1);
	nh_.param<int>("/robotino_vision_node/color/orange/thresh_1", orange_params_.thresh_1, 106);
	nh_.param<int>("/robotino_vision_node/color/orange/close_1", orange_params_.close_1, 0);
	nh_.param<int>("/robotino_vision_node/color/orange/open_1", orange_params_.open_1, 6);
	nh_.param<int>("/robotino_vision_node/color/orange/initial_range_value", orange_params_.initial_range_value, 241);
	nh_.param<int>("/robotino_vision_node/color/orange/range_width", orange_params_.range_width, 36);
	nh_.param<int>("/robotino_vision_node/color/orange/open_2", orange_params_.open_2, 0);
	nh_.param<int>("/robotino_vision_node/color/orange/close_2", orange_params_.close_2, 20);
	nh_.param<int>("/robotino_vision_node/color/orange/open_3", orange_params_.open_3, 5);
	nh_.param<int>("/robotino_vision_node/color/orange/min_area", orange_params_.min_area, MIN_AREA);

	ROS_DEBUG("********* Orange Color Parameters **********");
	ROS_DEBUG("~/orange/thresh_0: %d", orange_params_.thresh_0);
	ROS_DEBUG("~/orange/erosion_0: %d", orange_params_.erosion_0);
	ROS_DEBUG("~/orange/thresh_1: %d", orange_params_.thresh_1);
	ROS_DEBUG("~/orange/close_1: %d", orange_params_.close_1);
	ROS_DEBUG("~/orange/open_1: %d", orange_params_.open_1);
	ROS_DEBUG("~/orange/initial_range_value: %d", orange_params_.initial_range_value);
	ROS_DEBUG("~/orange/range_width: %d", orange_params_.range_width);
	ROS_DEBUG("~/orange/open_2: %d", orange_params_.open_2);
	ROS_DEBUG("~/orange/close_2: %d", orange_params_.close_2);
	ROS_DEBUG("~/orange/open_3: %d", orange_params_.open_3);
	ROS_DEBUG("~/orange/min_area: %d", orange_params_.min_area);

	nh_.param<int>("/robotino_vision_node/color/red/thresh_0", red_params_.thresh_0, 52);
	nh_.param<int>("/robotino_vision_node/color/red/erosion_0", red_params_.erosion_0, 2);
	nh_.param<int>("/robotino_vision_node/color/red/thresh_1", red_params_.thresh_1, 0);
	nh_.param<int>("/robotino_vision_node/color/red/close_1", red_params_.close_1, 0);
	nh_.param<int>("/robotino_vision_node/color/red/open_1", red_params_.open_1, 0);
	nh_.param<int>("/robotino_vision_node/color/red/initial_range_value", red_params_.initial_range_value, 245);
	nh_.param<int>("/robotino_vision_node/color/red/range_width", red_params_.range_width, 14);
	nh_.param<int>("/robotino_vision_node/color/red/open_2", red_params_.open_2, 3);
	nh_.param<int>("/robotino_vision_node/color/red/close_2", red_params_.close_2, 4);
	nh_.param<int>("/robotino_vision_node/color/red/open_3", red_params_.open_3, 2);
	nh_.param<int>("/robotino_vision_node/color/red/min_area", red_params_.min_area, MIN_AREA);

	ROS_DEBUG("********* Red Color Parameters **********");
	ROS_DEBUG("~/red/thresh_0: %d", red_params_.thresh_0);
	ROS_DEBUG("~/red/erosion_0: %d", red_params_.erosion_0);
	ROS_DEBUG("~/red/thresh_1: %d", red_params_.thresh_1);
	ROS_DEBUG("~/red/close_1: %d", red_params_.close_1);
	ROS_DEBUG("~/red/open_1: %d", red_params_.open_1);
	ROS_DEBUG("~/red/initial_range_value: %d", red_params_.initial_range_value);
	ROS_DEBUG("~/red/range_width: %d", red_params_.range_width);
	ROS_DEBUG("~/red/open_2: %d", red_params_.open_2);
	ROS_DEBUG("~/red/close_2: %d", red_params_.close_2);
	ROS_DEBUG("~/red/open_3: %d", red_params_.open_3);
	ROS_DEBUG("~/red/min_area: %d", red_params_.min_area);

	nh_.param<int>("/robotino_vision_node/color/green/thresh_0", green_params_.thresh_0, 36);
	nh_.param<int>("/robotino_vision_node/color/green/erosion_0", green_params_.erosion_0, 1);
	nh_.param<int>("/robotino_vision_node/color/green/thresh_1", green_params_.thresh_1, 55);
	nh_.param<int>("/robotino_vision_node/color/green/close_1", green_params_.close_1, 5);
	nh_.param<int>("/robotino_vision_node/color/green/open_1", green_params_.open_1, 5);
	nh_.param<int>("/robotino_vision_node/color/green/initial_range_value", green_params_.initial_range_value, 112);
	nh_.param<int>("/robotino_vision_node/color/green/range_width", green_params_.range_width, 40);
	nh_.param<int>("/robotino_vision_node/color/green/open_2", green_params_.open_2, 5);
	nh_.param<int>("/robotino_vision_node/color/green/close_2", green_params_.close_2, 0);
	nh_.param<int>("/robotino_vision_node/color/green/open_3", green_params_.open_3, 2);
	nh_.param<int>("/robotino_vision_node/color/green/min_area", green_params_.min_area, MIN_AREA);

	ROS_DEBUG("********* Green Color Parameters **********");
	ROS_DEBUG("~/green/thresh_0: %d", green_params_.thresh_0);
	ROS_DEBUG("~/green/erosion_0: %d", green_params_.erosion_0);
	ROS_DEBUG("~/green/thresh_1: %d", green_params_.thresh_1);
	ROS_DEBUG("~/green/close_1: %d", green_params_.close_1);
	ROS_DEBUG("~/green/open_1: %d", green_params_.open_1);
	ROS_DEBUG("~/green/initial_range_value: %d", green_params_.initial_range_value);
	ROS_DEBUG("~/green/range_width: %d", green_params_.range_width);
	ROS_DEBUG("~/green/open_2: %d", green_params_.open_2);
	ROS_DEBUG("~/green/close_2: %d", green_params_.close_2);
	ROS_DEBUG("~/green/open_3: %d", green_params_.open_3);
	ROS_DEBUG("~/green/min_area: %d", green_params_.min_area);

	nh_.param<int>("/robotino_vision_node/color/blue/thresh_0", blue_params_.thresh_0, 36);
	nh_.param<int>("/robotino_vision_node/color/blue/erosion_0", blue_params_.erosion_0, 1);
	nh_.param<int>("/robotino_vision_node/color/blue/thresh_1", blue_params_.thresh_1, 85);
	nh_.param<int>("/robotino_vision_node/color/blue/close_1", blue_params_.close_1, 0);
	nh_.param<int>("/robotino_vision_node/color/blue/open_1", blue_params_.open_1, 0);
	nh_.param<int>("/robotino_vision_node/color/blue/initial_range_value", blue_params_.initial_range_value, 150);
	nh_.param<int>("/robotino_vision_node/color/blue/range_width", blue_params_.range_width, 28);
	nh_.param<int>("/robotino_vision_node/color/blue/open_2", blue_params_.open_2, 1);
	nh_.param<int>("/robotino_vision_node/color/blue/close_2", blue_params_.close_2, 11);
	nh_.param<int>("/robotino_vision_node/color/blue/open_3", blue_params_.open_3, 2);
	nh_.param<int>("/robotino_vision_node/color/blue/min_area", blue_params_.min_area, MIN_AREA);

	ROS_DEBUG("********* Blue Color Parameters **********");
	ROS_DEBUG("~/blue/thresh_0: %d", blue_params_.thresh_0);
	ROS_DEBUG("~/blue/erosion_0: %d", blue_params_.erosion_0);
	ROS_DEBUG("~/blue/thresh_1: %d", blue_params_.thresh_1);
	ROS_DEBUG("~/blue/close_1: %d", blue_params_.close_1);
	ROS_DEBUG("~/blue/open_1: %d", blue_params_.open_1);
	ROS_DEBUG("~/blue/initial_range_value: %d", blue_params_.initial_range_value);
	ROS_DEBUG("~/blue/range_width: %d", blue_params_.range_width);
	ROS_DEBUG("~/blue/open_2: %d", blue_params_.open_2);
	ROS_DEBUG("~/blue/close_2: %d", blue_params_.close_2);
	ROS_DEBUG("~/blue/open_3: %d", blue_params_.open_3);
	ROS_DEBUG("~/blue/min_area: %d", blue_params_.min_area);

	nh_.param<int>("/robotino_vision_node/color/yellow/thresh_0", yellow_params_.thresh_0, 76);
	nh_.param<int>("/robotino_vision_node/color/yellow/erosion_0", yellow_params_.erosion_0, 1);
	nh_.param<int>("/robotino_vision_node/color/yellow/thresh_1", yellow_params_.thresh_1, 61);
	nh_.param<int>("/robotino_vision_node/color/yellow/close_1", yellow_params_.close_1, 0);
	nh_.param<int>("/robotino_vision_node/color/yellow/open_1", yellow_params_.open_1, 6);
	nh_.param<int>("/robotino_vision_node/color/yellow/initial_range_value", yellow_params_.initial_range_value, 22);
	nh_.param<int>("/robotino_vision_node/color/yellow/range_width", yellow_params_.range_width, 28);
	nh_.param<int>("/robotino_vision_node/color/yellow/open_2", yellow_params_.open_2, 3);
	nh_.param<int>("/robotino_vision_node/color/yellow/close_2", yellow_params_.close_2, 8);
	nh_.param<int>("/robotino_vision_node/color/yellow/open_3", yellow_params_.open_3, 0);
	nh_.param<int>("/robotino_vision_node/color/yellow/min_area", yellow_params_.min_area, MIN_AREA);

	ROS_DEBUG("********* Yellow Color Parameters **********");
	ROS_DEBUG("~/yellow/thresh_0: %d", yellow_params_.thresh_0);
	ROS_DEBUG("~/yellow/erosion_0: %d", yellow_params_.erosion_0);
	ROS_DEBUG("~/yellow/thresh_1: %d", yellow_params_.thresh_1);
	ROS_DEBUG("~/yellow/close_1: %d", yellow_params_.close_1);
	ROS_DEBUG("~/yellow/open_1: %d", yellow_params_.open_1);
	ROS_DEBUG("~/yellow/initial_range_value: %d", yellow_params_.initial_range_value);
	ROS_DEBUG("~/yellow/range_width: %d", yellow_params_.range_width);
	ROS_DEBUG("~/yellow/open_2: %d", yellow_params_.open_2);
	ROS_DEBUG("~/yellow/close_2: %d", yellow_params_.close_2);
	ROS_DEBUG("~/yellow/open_3: %d", yellow_params_.open_3);
	ROS_DEBUG("~/yellow/min_area: %d", yellow_params_.min_area);
}
/*

void RobotinoVision::processImageLampPost()
{
	cv::Mat imgBGR;
	cv::cvtColor(imgRGB_, imgBGR, CV_RGB2BGR);
	cv::imshow("BRG Model", imgBGR);
	cv::Mat imgHSL;
	cv::cvtColor(imgRGB_, imgHSL, CV_RGB2HLS);
	cv::imshow("HSL Model", imgHSL);
	cv::Mat splittedImgHSL[3];
	cv::split(imgHSL, splittedImgHSL);
	cv::Mat imgH = splittedImgHSL[0];
	cv::imshow("Hue", imgH);
	cv::Mat imgS = splittedImgHSL[1];
	cv::imshow("Saturation", imgS);
	cv::Mat imgL = splittedImgHSL[2];
	cv::imshow("Lightness", imgL);
	
	cv::Mat threshImgL;
	cv::namedWindow("Lightness Window", 1);
	cv::createTrackbar("Lightness threshold value:", "Lightness Window", &threshVal_, 255);
	int maxVal = 255;
	cv::threshold(imgL, threshImgL, threshVal_, maxVal, cv::THRESH_BINARY);
//	std::vector<std::vector<cv::Point>> contours;
//	std::vector<cv::Vec4i> hierarchy;
//	cv::findContours(threshImgL, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
//	cv::Mat imgBorders = cv::Mat::zeros(threshImgL.size(), CV_8U);
//	for (int i = 0; i < contours.size(); i++) {
//		cv::drawContours(imgBorders, contours, i, cv::Scalar(255), 2, 8, hierarchy, 0, cv::Point());
//	}
//	cv::imshow("Contours", imgBorders);
	//cv::imshow("Lightness Window", threshImgL);
	
	/*const int maskSize = 7;
	int mask[maskSize][maskSize] = {{0, 0, 0, 0, 0, 0, 0},
					{0, 0, 0, 0, 0, 0, 0},
					{0, 0, 0, 0, 0, 0, 0}, 
					{1, 1, 1, 0, 1, 1, 1},
					{0, 0, 0, 0, 0, 0, 0},
					{0, 0, 0, 0, 0, 0, 0}, 
					{0, 0, 0, 0, 0, 0, 0}};  
	cv::Mat kernel = cv::Mat(maskSize, maskSize, CV_8U, (void*) mask, maskSize);*/
/*	cv::namedWindow("Dilated Lightness", 1);
	cv::createTrackbar("Dilation cv::Size", "Dilated Lightness", &dilationSize_, 10);
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * dilationSize_ + 1, 2 * dilationSize_ + 1), Point(dilationSize_, dilationSize_)); */
	/*std::sstream == "";
	for (t_size i = 0; i < element.rows; i++)
	{
		for (t_size j = 0; j < element.columns; j++)
			s << 
		s << std::endl;
	}	*/
/*
	cv::Mat dilatedImgL;
	cv::dilate(threshImgL, dilatedImgL, element);
	cv::imshow("Dilated Lightness", dilatedImgL);
	
	cv::Mat mask = dilatedImgL - threshImgL;
	cv::imshow("Mask", mask);
	mask /= 255;*/
/*
	cv::Mat imgH1, imgH2; 
	imgH.copyTo(imgH1);//, CV_8U);
	cv::Scalar s1 = cv::Scalar(25);
	imgH1 += s1;
	//std::cout << imgH1;
	imgH.copyTo(imgH2);//, CV_8U;
	cv::Scalar s2 = cv::Scalar(230);
	imgH2 -= s2;
	cv::Mat newImgH = imgH1 + imgH2;*/
/*	std::vector<cv::Mat> masks;
	masks.push_back(mask);
	masks.push_back(mask);
	masks.push_back(mask);
	cv::Mat imgMask;
	cv::merge(masks, imgMask);
	cv::Mat maskedImgBGR;
	maskedImgBGR = imgBGR.mul(imgMask);
	cv::Mat finalImgH = imgH.mul(mask); 
	cv::imshow("Final Filter", maskedImgBGR);

	cv::Mat imgRed, imgRed1, imgRed2, imgRed3, imgRed4;
	//int redThreshMinVal1 = 0;
	//int redThreshMaxVal1 = 10;
	cv::namedWindow("Red Range", 1);
	cv::createTrackbar("Range1MinVal", "Red Range", &redThreshMinVal1_, 255);
	cv::createTrackbar("Range1MaxVal", "Red Range", &redThreshMaxVal1_, 255);
	int redMaxVal = 1;
	cv::threshold(finalImgH, imgRed1, redThreshMinVal1_, redMaxVal, cv::THRESH_BINARY);
	cv::threshold(finalImgH, imgRed2, redThreshMaxVal1_, redMaxVal, cv::THRESH_BINARY_INV);
	imgRed3 = imgRed1.mul(imgRed2); 
	//int redThreshMinVal2 = 228;
	//int redThreshMaxVal2 = 255;
	cv::createTrackbar("Range2MinVal", "Red Range", &redThreshMinVal2_, 255);
	cv::createTrackbar("Range2MaxVal", "Red Range", &redThreshMaxVal2_, 255);
	cv::threshold(finalImgH, imgRed1, redThreshMinVal2_, redMaxVal, cv::THRESH_BINARY);
	cv::threshold(finalImgH, imgRed2, redThreshMaxVal2_, redMaxVal, cv::THRESH_BINARY_INV);
	imgRed4 = imgRed1.mul(imgRed2);
	imgRed = (imgRed3 + imgRed4) * 255;
	cv::imshow("Red Range", imgRed);

	cv::Mat imgYellow, imgYellow1, imgYellow2;
	//int yellowThreshMinVal = 15;
	//int yellowThreshMaxVal = 55;
	cv::namedWindow("Yellow Range", 1);
	cv::createTrackbar("RangeMinVal", "Yellow Range", &yellowThreshMinVal_, 255);
	cv::createTrackbar("RangeMaxVal", "Yellow Range", &yellowThreshMaxVal_, 255);
	int yellowMaxVal = 1;
	cv::threshold(finalImgH, imgYellow1, yellowThreshMinVal_, yellowMaxVal, cv::THRESH_BINARY);
	cv::threshold(finalImgH, imgYellow2, yellowThreshMaxVal_, yellowMaxVal, cv::THRESH_BINARY_INV);
	imgYellow = imgYellow1.mul(imgYellow2) * 255;
	cv::imshow("Yellow Range", imgYellow);
	
	cv::Mat imgGreen, imgGreen1, imgGreen2;
	//int greenThreshMinVal = 58;
	//int greenThreshMaxVal = 96;
	cv::namedWindow("Green Range", 1);
	cv::createTrackbar("RangeMinVal", "Green Range", &greenThreshMinVal_, 255);
	cv::createTrackbar("RangeMaxVal", "Green Range", &greenThreshMaxVal_, 255);
	int greenMaxVal = 1;
	cv::threshold(finalImgH, imgGreen1, greenThreshMinVal_, greenMaxVal, cv::THRESH_BINARY);
	cv::threshold(finalImgH, imgGreen2, greenThreshMaxVal_, greenMaxVal, cv::THRESH_BINARY_INV);
	imgGreen = imgGreen1.mul(imgGreen2) * 255;
	cv::imshow("Green Range", imgGreen);
	
	cv::waitKey(80);
}*/

