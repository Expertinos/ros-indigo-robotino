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
	find_areas_srv_ = nh_.advertiseService("find_areas", &RobotinoVision::findAreas, this);
	get_list_srv_ = nh_.advertiseService("get_objects_list", &RobotinoVision::getList, this);
	get_lamp_posts_srv_ = nh_.advertiseService("get_lamp_posts", &RobotinoVision::getLampPosts, this);
	contain_in_list_srv_ = nh_.advertiseService("contain_in_list", &RobotinoVision::containInList, this);
	image_sub_ = it_.subscribe("image_raw", 1, &RobotinoVision::imageCallback, this);
	save_srv_ = nh_.advertiseService("save_image", &RobotinoVision::saveImage, this);
	set_calibration_srv_ = nh_.advertiseService("set_calibration", &RobotinoVision::setCalibration, this);

	imgRGB_ = cv::Mat(width_, height_, CV_8UC3, cv::Scalar::all(0));

	setColor();	
	contours_window_name_ = CONTOURS_WINDOW + ": " + Colors::toString(color_);		

	calibration_ = false;
	
	verify_markers_ = false;
	specific_number_of_markers_ = 0;
	
	//setImagesWindows();

	/* Possible combinations
	colors::GREEN, colors::GREEN
	colors::YELLOW, colors::YELLOW
	colors::RED, colors::RED
	colors::GREEN, colors::YELLOW
	colors::GREEN, colors::RED
	colors::YELLOW, colors::RED
	colors::YELLOW, colors::GREEN
	colors::RED, colors::GREEN*/	
	lamp_posts_.resize(8);
	lamp_posts_[0].left.green = true; lamp_posts_[0].right.green = true;
	lamp_posts_[0].left.yellow = false; lamp_posts_[0].right.yellow = false;
	lamp_posts_[0].left.red = false; lamp_posts_[0].right.red = false;
	lamp_posts_[1].left.green = false; lamp_posts_[1].right.green = false;
	lamp_posts_[1].left.yellow = true; lamp_posts_[1].right.yellow = true;
	lamp_posts_[1].left.red = false; lamp_posts_[1].right.red = false;
	lamp_posts_[2].left.green = false; lamp_posts_[2].right.green = false;
	lamp_posts_[2].left.yellow = false; lamp_posts_[2].right.yellow = false;
	lamp_posts_[2].left.red = true; lamp_posts_[2].right.red = true;
	lamp_posts_[3].left.green = true; lamp_posts_[3].right.green = false;
	lamp_posts_[3].left.yellow = false; lamp_posts_[3].right.yellow = true;
	lamp_posts_[3].left.red = false; lamp_posts_[3].right.red = false;
	lamp_posts_[4].left.green = true; lamp_posts_[4].right.green = false;
	lamp_posts_[4].left.yellow = false; lamp_posts_[4].right.yellow = false;
	lamp_posts_[4].left.red = false; lamp_posts_[4].right.red = true;
	lamp_posts_[5].left.green = false; lamp_posts_[5].right.green = false;
	lamp_posts_[5].left.yellow = true; lamp_posts_[5].right.yellow = false;
	lamp_posts_[5].left.red = false; lamp_posts_[5].right.red = true;
	lamp_posts_[6].left.green = false; lamp_posts_[6].right.green = true;
	lamp_posts_[6].left.yellow = true; lamp_posts_[6].right.yellow = false;
	lamp_posts_[6].left.red = false; lamp_posts_[6].right.red = false;
	lamp_posts_[7].left.green = false; lamp_posts_[7].right.green = true;
	lamp_posts_[7].left.yellow = false; lamp_posts_[7].right.yellow = false;
	lamp_posts_[7].left.red = true; lamp_posts_[7].right.red = false;
	srand(time(NULL));

	color_lamp_dilate_ = 6;
	color_lamp_close_ = 4;
}

/**
 *
 */
double RobotinoVision::randomDouble(double min, double max) {
	return (max - min) * (rand() % 101) / 100 + min;
}

/**
 *
 */
int RobotinoVision::randomInteger(int min, int max) {
	return (max - min) * (rand() % 101) / 100 + min;
}


/**
 *
 */
RobotinoVision::~RobotinoVision()
{
	imgRGB_.release();
	find_objects_srv_.shutdown();
	find_areas_srv_.shutdown();
	get_list_srv_.shutdown();
	get_lamp_posts_srv_.shutdown();
	contain_in_list_srv_.shutdown();
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
		if (!number_of_markers_.empty())
		{
			res.number_of_markers.push_back(number_of_markers_[k]);
		}
	}
	return true;
}

/**
 *
 */
bool RobotinoVision::findAreas(robotino_vision::FindInsulatingTapeAreas::Request &req, robotino_vision::FindInsulatingTapeAreas::Response &res)
{
	ROS_DEBUG("Getting Insulating Tape Area!!!");
	cv::Mat insulating_tape_area = getInsulatingTapeArea();
	verify_markers_ = false;
	std::vector<cv::Point2f> mass_center = getContours(insulating_tape_area);
	std::vector<cv::Point2f> positions = getPositions(mass_center);
	int number_of_areas = positions.size();
	for (int k = 0; k < number_of_areas; k++)
	{
		res.distances.push_back(positions[k].x);
		res.directions.push_back(positions[k].y);
	}
	insulating_tape_area.release();
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
bool RobotinoVision::getLampPosts(robotino_vision::GetLampPosts::Request &req, robotino_vision::GetLampPosts::Response &res)
{
	if (lamp_posts_.empty())
	{
		res.success = false;
		return true;
	}
	int index = randomInteger(0, lamp_posts_.size() - 1);
	res.left = lamp_posts_[index].left;
	res.right = lamp_posts_[index].right;
	lamp_posts_.erase(lamp_posts_.begin() + index);
	res.success = true;
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
void RobotinoVision::updateParameters()
{
	std::string contours_window_name = CONTOURS_WINDOW + ": " + Colors::toString(color_);
	if (contours_window_name != contours_window_name_) 
	{
		contours_window_name_ = contours_window_name;
		without_markers_window_name_ = Colors::toString(color_) + " " + PUCK_WITHOUT_MARKERS_WINDOW;
		setImagesWindows();
	}	

	cv::createTrackbar("Close: ", LIGHTING_MASK_WINDOW, &lighting_close_, 20);
	cv::createTrackbar("Blur: ", LIGHTING_MASK_WINDOW, &lighting_blur_size_, 20);
	cv::createTrackbar("Threshold: ", LIGHTING_MASK_WINDOW, &lighting_thresh_, 255);
	cv::createTrackbar("Open: ", LIGHTING_MASK_WINDOW, &lighting_open_, 20);
	cv::createTrackbar("Dilate: ", LIGHTING_MASK_WINDOW, &lighting_dilate_, 20);

	cv::createTrackbar("Blur: ", ALL_MARKERS_WINDOW, &markers_blur_size_, 20);
	cv::createTrackbar("Threshold: ", ALL_MARKERS_WINDOW, &markers_thresh_, 255);
	cv::createTrackbar("Open: ", ALL_MARKERS_WINDOW, &markers_open_, 20);

	cv::createTrackbar("Blur: ", PUCKS_MASK_WINDOW, &pucks_blur_size_, 40);
	cv::createTrackbar("Dilate: ", PUCKS_MASK_WINDOW, &pucks_dilate_, 40);
	cv::createTrackbar("Threshold: ", PUCKS_MASK_WINDOW, &pucks_thresh_, 255);
	cv::createTrackbar("Close: ", PUCKS_MASK_WINDOW, &pucks_close_, 40);
	cv::createTrackbar("Open: ", PUCKS_MASK_WINDOW, &pucks_open_, 40);

	cv::createTrackbar("Initial range value: ", COLOR_MASK_WINDOW, &color_params_.initial_range_value, 255);
	cv::createTrackbar("Range width: ", COLOR_MASK_WINDOW, &color_params_.range_width, 255);
	cv::createTrackbar("Blur: ", COLOR_MASK_WINDOW, &color_blur_size_, 40);
	cv::createTrackbar("Dilate: ", COLOR_MASK_WINDOW, &color_dilate_, 40);
	cv::createTrackbar("Open: ", COLOR_MASK_WINDOW, &color_open_, 40);
	cv::createTrackbar("Close: ", COLOR_MASK_WINDOW, &color_close_, 40);

	cv::createTrackbar("Open(before): ", FINAL_MASK_WINDOW, &final_open_before_, 20);
	cv::createTrackbar("Close: ", FINAL_MASK_WINDOW, &final_close_, 20);
	cv::createTrackbar("Open(after): ", FINAL_MASK_WINDOW, &final_open_after_, 20);
	cv::createTrackbar("Dilate: ", FINAL_MASK_WINDOW, &final_dilate_, 20);

	cv::createTrackbar("Blur: ", INSULATING_TAPE_WINDOW, &area_blur_size_, 50);
	cv::createTrackbar("Threshold: ", INSULATING_TAPE_WINDOW, &area_thresh_, 255);
	cv::createTrackbar("Close: ", INSULATING_TAPE_WINDOW, &area_close_, 50);
	cv::createTrackbar("Dilate: ", INSULATING_TAPE_WINDOW, &area_dilate_, 50);
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

	ROS_DEBUG("Getting Lighting Lamp Mask!!!");
	cv::Mat lighting_mask = getLightingLampsMask();
	ROS_DEBUG("Getting Pucks Mask!!!");
	cv::Mat pucks_mask = getPucksMask();
	ROS_DEBUG("Getting Color Mask!!!");
	cv::Mat color_mask = getColorMask();
	ROS_DEBUG("Getting Lighting Lamp Mask!!!");
	cv::Mat color_lamp_mask = getLightingColorLampsMask(lighting_mask, color_mask);
	ROS_DEBUG("Getting Final Puck Mask!!!");
	cv::Mat final_mask = getFinalMask(pucks_mask, color_mask);
	ROS_DEBUG("Getting All Markers!!!");
	cv::Mat all_markers = getAllMarkers(pucks_mask);
	ROS_DEBUG("Getting Puck Markers!!!");
	cv::Mat puck_markers = getPuckMarkers(all_markers, pucks_mask);
	ROS_DEBUG("Getting Puck without Markers!!!");
	cv::Mat puck_without_markers = getPuckWithoutMarkers(final_mask, puck_markers);
	ROS_DEBUG("Getting Contours based on Final Mask!!!");
	cv::Mat area = getInsulatingTapeArea();

	if (calibration_)
	{
		updateParameters();
		cv::imshow(INSULATING_TAPE_WINDOW, area);
		cv::imshow(PUCK_MARKERS_WINDOW, puck_markers);
		showImageBGRwithMask(final_mask);
	}

	cv::createTrackbar("Blur: ", LIGHTING_MASK_WINDOW, &lighting_blur_size_, 20);
	cv::createTrackbar("Threshold: ", LIGHTING_MASK_WINDOW, &lighting_thresh_, 255);
	cv::createTrackbar("Open: ", LIGHTING_MASK_WINDOW, &lighting_open_, 20);
	cv::createTrackbar("Close: ", LIGHTING_MASK_WINDOW, &lighting_close_, 20);
	cv::createTrackbar("Dilate: ", LIGHTING_MASK_WINDOW, &lighting_dilate_, 20);
	cv::imshow(LIGHTING_MASK_WINDOW, lighting_mask);

	cv::createTrackbar("Dilate: ", COLOR_LAMP_MASK_WINDOW, &color_lamp_dilate_, 40);
	cv::createTrackbar("Close: ", COLOR_LAMP_MASK_WINDOW, &color_lamp_close_, 20);
	cv::imshow(COLOR_LAMP_MASK_WINDOW, color_lamp_mask);

	cv::createTrackbar("Initial range value: ", COLOR_MASK_WINDOW, &color_params_.initial_range_value, 255);
	cv::createTrackbar("Range width: ", COLOR_MASK_WINDOW, &color_params_.range_width, 255);
	cv::createTrackbar("Blur: ", COLOR_MASK_WINDOW, &color_blur_size_, 40);
	cv::createTrackbar("Dilate: ", COLOR_MASK_WINDOW, &color_dilate_, 40);
	cv::createTrackbar("Open: ", COLOR_MASK_WINDOW, &color_open_, 40);
	cv::createTrackbar("Close: ", COLOR_MASK_WINDOW, &color_close_, 40);
	cv::imshow(COLOR_MASK_WINDOW, color_mask);

	lighting_mask.release();
	all_markers.release();
	pucks_mask.release();
	color_mask.release();
	color_lamp_mask.release();
	puck_markers.release();

	points = getContours(puck_without_markers);//final_mask);

	final_mask.release();
	puck_without_markers.release();
	area.release();
	
	cv::waitKey(30);

	return points;
}

/**
 *
 */
cv::Mat RobotinoVision::getLightingLampsMask() 
{
	cv::Mat  lighting_mask;

	// convertendo de RGB para HLS
	cv::Mat imgHLS;
	cv::cvtColor(imgRGB_, imgHLS, CV_RGB2HLS);

	// separando a HLS 
	cv::Mat splitted[3];
	cv::split(imgHLS, splitted);
	 lighting_mask = splitted[2];

	// fechando buracos na area
	cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(2 * lighting_close_ + 1, 2 * lighting_close_ + 1), cv::Point(lighting_close_, lighting_close_));
	cv::morphologyEx( lighting_mask,  lighting_mask, cv::MORPH_CLOSE, element);

	cv::medianBlur(lighting_mask, lighting_mask, 2 * lighting_blur_size_ + 1);

	// define o intervalo da cor
	cv::threshold(lighting_mask, lighting_mask, lighting_thresh_, 255, cv::THRESH_BINARY);

	// filtrando particulas pequenas
	element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * lighting_open_ + 1, 2 * lighting_open_ + 1), cv::Point(lighting_open_, lighting_open_));
	cv::morphologyEx( lighting_mask,  lighting_mask, cv::MORPH_OPEN, element);

	// fazendo dilatação na imagem acima
	element = getStructuringElement(cv::MORPH_RECT, cv::Size(2 * lighting_dilate_ + 1, 2 * lighting_dilate_ + 1), cv::Point(lighting_dilate_, lighting_dilate_));
	cv::dilate(lighting_mask, lighting_mask, element);

	if (calibration_)
	{
		cv::imshow(LIGHTING_MASK_WINDOW,  lighting_mask);
	}

	imgHLS.release();
	splitted[0].release();
	splitted[1].release();
	splitted[2].release();
	element.release();

	return  lighting_mask;
}

/**
 *
 */	
cv::Mat RobotinoVision::getLightingColorLampsMask(cv::Mat &lighting_mask, cv::Mat & color_mask)
{
	cv::Mat color_lamp_mask;
	cv::bitwise_and(lighting_mask, color_mask, color_lamp_mask);
	cv::imshow("Color Light Lamp", color_lamp_mask);

	// fazendo dilatação na imagem acima
	cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(2 * color_lamp_dilate_ + 1, 2 * color_lamp_dilate_ + 1), cv::Point(color_lamp_dilate_, color_lamp_dilate_));
	cv::dilate(color_lamp_mask,  color_lamp_mask, element);
	cv::imshow("Color Light Lamp dilate", color_lamp_mask);

	// fechando buracos na area
	element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(2 * color_lamp_close_ + 1, 2 * color_lamp_close_ + 1), cv::Point(color_lamp_close_, color_lamp_close_));
	cv::morphologyEx(color_lamp_mask, color_lamp_mask, cv::MORPH_CLOSE, element);

	if (calibration_)
	{
		cv::imshow(COLOR_LAMP_MASK_WINDOW, color_lamp_mask);
	}

	element.release();
	
	return color_lamp_mask;
}

/**
 *
 */
cv::Mat RobotinoVision::getAllMarkers(cv::Mat &pucks_mask)
{
	cv::Mat all_markers;

	// convertendo de RGB para HSV
	cv::Mat imgHSV;
	cv::cvtColor(imgRGB_, imgHSV, CV_RGB2HSV);

	// separando a HSV 
	cv::Mat splitted[3];
	cv::split(imgHSV, splitted);
	all_markers = splitted[2];

	cv::medianBlur(all_markers, all_markers, 2 * markers_blur_size_ + 1);

	// fazendo threshold da imagem V
	cv::threshold(all_markers, all_markers, markers_thresh_, 255, cv::THRESH_BINARY);

	// filtro de partícula pequenas
	cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(2 * markers_open_ + 1, 2 * markers_open_ + 1), cv::Point(markers_open_, markers_open_));
	cv::morphologyEx(all_markers, all_markers, cv::MORPH_OPEN, element);

	if (calibration_)
	{
		cv::imshow(ALL_MARKERS_WINDOW, all_markers);
	}

	imgHSV.release();
	splitted[0].release();
	splitted[1].release();
	splitted[2].release();
	element.release();

	return all_markers;
}

/**
 *
 */
cv::Mat RobotinoVision::getInsulatingTapeArea()
{
	cv::Mat insulating_tape_area;

	// convertendo de RGB para HSV
	cv::Mat imgHSV;
	cv::cvtColor(imgRGB_, imgHSV, CV_RGB2HSV);

	// separando a HSV 
	cv::Mat splitted[3];
	cv::split(imgHSV, splitted);
	insulating_tape_area = splitted[2];

	cv::medianBlur(insulating_tape_area, insulating_tape_area, 2 * area_blur_size_ + 1);

	// fazendo threshold da imagem V
	cv::threshold(insulating_tape_area, insulating_tape_area, area_thresh_, 255, cv::THRESH_BINARY);

	// invertendo imagem
	insulating_tape_area = 255 - insulating_tape_area;

	// fazendo dilatação na imagem acima
	cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(2 * area_dilate_ + 1, 2 * area_dilate_ + 1), cv::Point(area_dilate_, area_dilate_));
	cv::dilate(insulating_tape_area, insulating_tape_area, element);

	// fechando buracos na area
	element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(2 * area_close_ + 1, 2 * area_close_ + 1), cv::Point(area_close_, area_close_));
	cv::morphologyEx(insulating_tape_area, insulating_tape_area, cv::MORPH_CLOSE, element);

	// fazendo dilatação na imagem acima
	element = getStructuringElement(cv::MORPH_RECT, cv::Size(2 * area_dilate_ + 1, 2 * area_dilate_ + 1), cv::Point(area_dilate_, area_dilate_));
	cv::dilate(insulating_tape_area, insulating_tape_area, element);
	
	if (calibration_)
	{
		cv::imshow(INSULATING_TAPE_WINDOW, insulating_tape_area);
	}

	imgHSV.release();
	splitted[0].release();
	splitted[1].release();
	splitted[2].release();
	element.release();

	return insulating_tape_area;
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

	cv::medianBlur(pucks_mask, pucks_mask, 2 * pucks_blur_size_ + 1);

	// fazendo dilatação na imagem acima
	cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(2 * pucks_dilate_ + 1, 2 * pucks_dilate_ + 1), cv::Point(pucks_dilate_, pucks_dilate_));
	cv::dilate(pucks_mask, pucks_mask, element);

	// fazendo threshold da imagem S
	cv::threshold(pucks_mask, pucks_mask, pucks_thresh_, 255, cv::THRESH_BINARY);

	// fechando buracos
	element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(2 * pucks_close_ + 1, 2 * pucks_close_ + 1), cv::Point(pucks_close_, pucks_close_));
	cv::morphologyEx(pucks_mask, pucks_mask, cv::MORPH_CLOSE, element);

	// filtro de partícula pequenas
	element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(2 * pucks_open_ + 1, 2 * pucks_open_ + 1), cv::Point(pucks_open_, pucks_open_));
	cv::morphologyEx(pucks_mask, pucks_mask, cv::MORPH_OPEN, element);

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
	color_mask = splitted[0];

	cv::medianBlur(color_mask, color_mask, 2 * color_blur_size_ + 1);

	color_mask = 1.41666 * color_mask;

	// rodando a roleta da imagem H com blur
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

	// fazendo dilatação na imagem acima
	cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(2 * color_dilate_ + 1, 2 * color_dilate_ + 1), cv::Point(color_dilate_, color_dilate_));
	cv::dilate(color_mask, color_mask, element);

	// filtro de partícula pequenas
	element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(2 * color_open_ + 1, 2 * color_open_ + 1), cv::Point(color_open_, color_open_));
	cv::morphologyEx(color_mask, color_mask, cv::MORPH_OPEN, element);

	// fechando buracos
	element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(2 * color_close_ + 1, 2 * color_close_ + 1), cv::Point(color_close_, color_close_));
	cv::morphologyEx(color_mask, color_mask, cv::MORPH_CLOSE, element);

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
cv::Mat RobotinoVision::getFinalMask(cv::Mat &insulating_tape_area, cv::Mat &pucks_mask, cv::Mat &color_mask)
{
	// juntando todas as máscaras
	cv::Mat final_mask;
	cv::bitwise_and(pucks_mask, insulating_tape_area, final_mask);
	cv::bitwise_and(final_mask, color_mask, final_mask);

	// removendo particulas pequenas
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * final_open_before_ + 1, 2 * final_open_before_ + 1), cv::Point(final_open_before_, final_open_before_));
	cv::morphologyEx(final_mask, final_mask, cv::MORPH_OPEN, element);
	
	// fechando buracos
	element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * final_close_ + 1, 2 * final_close_ + 1), cv::Point(final_close_, final_close_));
	morphologyEx(final_mask, final_mask, cv::MORPH_CLOSE, element);

	// removendo particulas pequenas
	element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * final_open_after_ + 1, 2 * final_open_after_ + 1), cv::Point(final_open_after_, final_open_after_));
	cv::morphologyEx(final_mask, final_mask, cv::MORPH_OPEN, element);

	// fazendo dilatação na imagem acima
	element = getStructuringElement(cv::MORPH_RECT, cv::Size(2 * final_dilate_+ 1, 2 * final_dilate_ + 1), cv::Point(final_dilate_, final_dilate_));
	cv::dilate(final_mask, final_mask, element);

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
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * final_open_before_ + 1, 2 * final_open_before_ + 1), cv::Point(final_open_before_, final_open_before_));
	cv::morphologyEx(final_mask, final_mask, cv::MORPH_OPEN, element);
	
	// fechando buracos
	element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * final_close_ + 1, 2 * final_close_ + 1), cv::Point(final_close_, final_close_));
	morphologyEx(final_mask, final_mask, cv::MORPH_CLOSE, element);

	// removendo particulas pequenas
	element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * final_open_after_ + 1, 2 * final_open_after_ + 1), cv::Point(final_open_after_, final_open_after_));
	cv::morphologyEx(final_mask, final_mask, cv::MORPH_OPEN, element);

	// fazendo dilatação na imagem acima
	element = getStructuringElement(cv::MORPH_RECT, cv::Size(2 * final_dilate_+ 1, 2 * final_dilate_ + 1), cv::Point(final_dilate_, final_dilate_));
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
cv::Mat RobotinoVision::getPuckMarkers(cv::Mat &insulating_tape_area, cv::Mat &pucks_mask)
{
	// juntando todas as máscaras
	cv::Mat puck_markers;
	cv::bitwise_and(pucks_mask, insulating_tape_area, puck_markers);
	
	// filtro de partícula pequenas
	cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(2 * markers_open_ + 1, 2 * markers_open_ + 1), cv::Point(markers_open_, markers_open_));
	cv::morphologyEx(puck_markers, puck_markers, cv::MORPH_OPEN, element);

	element.release();
	
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
		if (!verify_markers_ && area > MIN_AREA)
		{
			contours.push_back(initial_contours[i]);
		} 
		else if (verify_markers_ && area > 25)
		{
			bool has_parent = hierarchy[i][3] != -1;
			if (!has_parent && area > MIN_AREA)
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
		/*if (area < MIN_AREA)
		{
			contours.erase(contours.begin() + i);
			if (verify_markers_)
			{
				number_of_markers_.erase(number_of_markers_.begin() + i);
			}
		}*/
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
		contours_window_name_ = contours_window_name;
		without_markers_window_name_ = Colors::toString(color_) + " " + PUCK_WITHOUT_MARKERS_WINDOW;
		setImagesWindows();
	}
	cv::imshow(contours_window_name_.c_str(), drawing);

	int numberOfObjects = mass_centers.size();
	if (numberOfObjects > MAX_NUMBER_OF_PUCKS) 
	{
		ROS_DEBUG("You must recalibrate the %s color parameters, there are too much noise!!!", Colors::toString(color_).c_str());
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
	cv::namedWindow(LIGHTING_MASK_WINDOW);
	cv::namedWindow(ALL_MARKERS_WINDOW);
	cv::namedWindow(PUCKS_MASK_WINDOW);
	cv::namedWindow(COLOR_MASK_WINDOW);
	cv::namedWindow(FINAL_MASK_WINDOW);
	cv::namedWindow(PUCK_MARKERS_WINDOW);
	cv::namedWindow(contours_window_name_.c_str());
	cv::namedWindow(INSULATING_TAPE_WINDOW);
	cv::namedWindow(BGR_WINDOW);
	cv::moveWindow(ALL_MARKERS_WINDOW, 0.5 * width_, 0 * height_);
	cv::moveWindow(PUCKS_MASK_WINDOW, 1.5 * width_, 0 * height_);
	cv::moveWindow(COLOR_MASK_WINDOW, 2.5 * width_, 0 * height_);
	cv::moveWindow(FINAL_MASK_WINDOW, 3.5 * width_, 0 * height_);	
	cv::moveWindow(INSULATING_TAPE_WINDOW, 4.5 * width_, 0 * height_);
	cv::moveWindow(PUCK_MARKERS_WINDOW, 0.5 * width_, 3 * height_);
	cv::moveWindow(contours_window_name_.c_str(), 1.5 * width_, 3 * height_);
	cv::moveWindow(BGR_WINDOW, 2.5 * width_, 3 * height_);
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
	ROS_DEBUG("************* Camera Position ************* ");
	nh_.param<double>("/robotino_vision_node/camera/height", camera_height_, 28.0); // in centimeters
	nh_.param<double>("/robotino_vision_node/camera/close_distance", camera_close_distance_, 20.0); // in centimeters
	nh_.param<double>("/robotino_vision_node/camera/far_distance", camera_far_distance_, 62.0); // in centimeters
	nh_.param<double>("/robotino_vision_node/camera/depth_width", camera_depth_width_, 50.0); // in centimeters
	ROS_DEBUG("~/camera/height: %f", camera_height_); 
	ROS_DEBUG("~/camera/close_distance: %f", camera_close_distance_);
	ROS_DEBUG("~/camera/far_distance: %f", camera_far_distance_);
	ROS_DEBUG("~/camera/depth_width: %f", camera_depth_width_);

	ROS_DEBUG("************ Image Size ***************** ");
	nh_.param<int>("/robotino_vision_node/image/height", height_, 240); // in pixels
	nh_.param<int>("/robotino_vision_node/image/width", width_, 320); // in pixels
	ROS_DEBUG("~/image/height: %d", height_);
	ROS_DEBUG("~/image/width: %d", width_);

	// which were defined in config/color_params.yaml file
	ROS_DEBUG("********* Orange Color Parameters **********");
	nh_.param<int>("/robotino_vision_node/color/orange/initial_range_value", orange_params_.initial_range_value, 241);
	nh_.param<int>("/robotino_vision_node/color/orange/range_width", orange_params_.range_width, 36);
	ROS_DEBUG("~/orange/initial_range_value: %d", orange_params_.initial_range_value);
	ROS_DEBUG("~/orange/range_width: %d", orange_params_.range_width);

	ROS_DEBUG("********* Red Color Parameters **********");
	nh_.param<int>("/robotino_vision_node/color/red/initial_range_value", red_params_.initial_range_value, 245);
	nh_.param<int>("/robotino_vision_node/color/red/range_width", red_params_.range_width, 14);
	ROS_DEBUG("~/red/initial_range_value: %d", red_params_.initial_range_value);
	ROS_DEBUG("~/red/range_width: %d", red_params_.range_width);

	ROS_DEBUG("********* Green Color Parameters **********");
	nh_.param<int>("/robotino_vision_node/color/green/initial_range_value", green_params_.initial_range_value, 112);
	nh_.param<int>("/robotino_vision_node/color/green/range_width", green_params_.range_width, 40);
	ROS_DEBUG("~/green/initial_range_value: %d", green_params_.initial_range_value);
	ROS_DEBUG("~/green/range_width: %d", green_params_.range_width);

	ROS_DEBUG("********* Blue Color Parameters **********");
	nh_.param<int>("/robotino_vision_node/color/blue/initial_range_value", blue_params_.initial_range_value, 150);
	nh_.param<int>("/robotino_vision_node/color/blue/range_width", blue_params_.range_width, 28);
	ROS_DEBUG("~/blue/initial_range_value: %d", blue_params_.initial_range_value);
	ROS_DEBUG("~/blue/range_width: %d", blue_params_.range_width);

	ROS_DEBUG("********* Yellow Color Parameters **********");
	nh_.param<int>("/robotino_vision_node/color/yellow/initial_range_value", yellow_params_.initial_range_value, 22);
	nh_.param<int>("/robotino_vision_node/color/yellow/range_width", yellow_params_.range_width, 28);
	ROS_DEBUG("~/yellow/initial_range_value: %d", yellow_params_.initial_range_value);
	ROS_DEBUG("~/yellow/range_width: %d", yellow_params_.range_width);

	// which were defined in config/image_processing_params.yaml file
	ROS_DEBUG("********* All Markers Mask Parameters **********");
	nh_.param<int>("/robotino_vision_node/mask/markers/blur_size", markers_blur_size_, 1);
	nh_.param<int>("/robotino_vision_node/mask/markers/thresh", markers_thresh_, 220);
	nh_.param<int>("/robotino_vision_node/mask/markers/open", markers_open_, 2);
	ROS_DEBUG("~/mask/markers/blur_size: %d", markers_blur_size_);
	ROS_DEBUG("~/mask/markers/thresh: %d", markers_thresh_);
	ROS_DEBUG("~/mask/markers/open: %d", markers_open_);

	ROS_DEBUG("********* All Pucks Mask Parameters **********");
	nh_.param<int>("/robotino_vision_node/mask/pucks/blur_size", pucks_blur_size_, 4);
	nh_.param<int>("/robotino_vision_node/mask/pucks/dilate", pucks_dilate_, 6);
	nh_.param<int>("/robotino_vision_node/mask/pucks/thresh", pucks_thresh_, 70);
	nh_.param<int>("/robotino_vision_node/mask/pucks/close", pucks_close_, 2);
	nh_.param<int>("/robotino_vision_node/mask/pucks/open", pucks_open_, 2);
	ROS_DEBUG("~/mask/pucks/blur_size: %d", pucks_blur_size_);
	ROS_DEBUG("~/mask/pucks/dilate: %d", pucks_dilate_);
	ROS_DEBUG("~/mask/pucks/thresh: %d", pucks_thresh_);
	ROS_DEBUG("~/mask/pucks/close: %d", pucks_close_);
	ROS_DEBUG("~/mask/pucks/open: %d", pucks_open_);

	ROS_DEBUG("********* Color Mask Parameters **********");
	nh_.param<int>("/robotino_vision_node/mask/color/blur_size", color_blur_size_, 3);
	nh_.param<int>("/robotino_vision_node/mask/color/dilate", color_dilate_, 2);
	nh_.param<int>("/robotino_vision_node/mask/color/open", color_open_, 2);
	nh_.param<int>("/robotino_vision_node/mask/color/close", color_close_, 5);
	ROS_DEBUG("~/mask/color/blur_size: %d", color_blur_size_);
	ROS_DEBUG("~/mask/color/dilate: %d", color_dilate_);
	ROS_DEBUG("~/mask/color/open: %d", color_open_);
	ROS_DEBUG("~/mask/color/close: %d", color_close_);

	ROS_DEBUG("********* Final Puck Mask Parameters **********");
	nh_.param<int>("/robotino_vision_node/mask/final_puck/open/before", final_open_before_, 2);
	nh_.param<int>("/robotino_vision_node/mask/final_puck/close", final_close_, 2);
	nh_.param<int>("/robotino_vision_node/mask/final_puck/open/after", final_open_after_, 7);
	nh_.param<int>("/robotino_vision_node/mask/final_puck/dilate", final_dilate_, 2);
	ROS_DEBUG("~/mask/final_puck/open/before: %d", final_open_before_);
	ROS_DEBUG("~/mask/final_puck/close: %d", final_close_);
	ROS_DEBUG("~/mask/final_puck/open/after: %d", final_open_after_);
	ROS_DEBUG("~/mask/final_puck/dilate: %d", final_dilate_);

	ROS_DEBUG("********* Insulating Tape Area Mask Parameters **********");
	nh_.param<int>("/robotino_vision_node/mask/insulating_tape_area/blur_size", area_blur_size_, 1);
	nh_.param<int>("/robotino_vision_node/mask/insulating_tape_area/thresh", area_thresh_, 50);
	nh_.param<int>("/robotino_vision_node/mask/insulating_tape_area/close", area_close_, 15);
	nh_.param<int>("/robotino_vision_node/mask/insulating_tape_area/dilate", area_dilate_, 15);
	ROS_DEBUG("~/mask/insulating_tape_area/blur_size: %d", area_blur_size_);
	ROS_DEBUG("~/mask/insulating_tape_area/thresh: %d", area_thresh_);
	ROS_DEBUG("~/mask/insulating_tape_area/close: %d", area_close_);
	ROS_DEBUG("~/mask/insulating_tape_area/dilate: %d", area_dilate_);

	ROS_DEBUG("********* Lighting Lamp Mask Parameters **********");
	nh_.param<int>("/robotino_vision_node/mask/lighting_lamp/close", lighting_close_, 5);
	nh_.param<int>("/robotino_vision_node/mask/lighting_lamp/blur_size", lighting_blur_size_, 2);
	nh_.param<int>("/robotino_vision_node/mask/lighting_lamp/thresh", lighting_thresh_, 235);
	nh_.param<int>("/robotino_vision_node/mask/lighting_lamp/open", lighting_open_, 7);
	nh_.param<int>("/robotino_vision_node/mask/lighting_lamp/dilate", lighting_dilate_, 2);
	ROS_DEBUG("~/mask/lighting_lamp/close: %d", lighting_close_);
	ROS_DEBUG("~/mask/lighting_lamp/blur_size: %d", lighting_blur_size_);
	ROS_DEBUG("~/mask/lighting_lamp/thresh: %d", lighting_thresh_);
	ROS_DEBUG("~/mask/lighting_lamp/open: %d", lighting_open_);
	ROS_DEBUG("~/mask/lighting_lamp/dilate: %d", lighting_dilate_);
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

