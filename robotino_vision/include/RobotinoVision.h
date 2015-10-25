/*
 * RobotinoVision.h
 *
 *  Created on: 15/07/2014
 *      Author: adrianohrl@unifei.edu.br
 */

#ifndef RobotinoVision_H
#define RobotinoVision_H

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#include <vector>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <fl/Headers.h>

#include "Colors.h"
#include "robotino_vision/FindObjects.h"
#include "robotino_vision/FindInsulatingTapeAreas.h"
#include "robotino_vision/GetObjectsList.h"
#include "robotino_vision/LampPost.h"
#include "robotino_vision/GetLampPosts.h"
#include "robotino_vision/ContainInList.h"
#include "robotino_vision/SaveImage.h"
#include "robotino_vision/SetCalibration.h"

#define MAX_NUMBER_OF_PUCKS 2
#define MIN_AREA 500 //1000 //a fim de eliminar os chofiscos no final do processamento das imagens
#define PI 3.14159

static const std::string INSULATING_TAPE_WINDOW = "Insulating Tape Window";
static const std::string ALL_MARKERS_WINDOW = "All Markers Window";
static const std::string PUCKS_MASK_WINDOW = "Pucks Mask Window";
static const std::string COLOR_MASK_WINDOW = "Color Mask Window";
static const std::string FINAL_MASK_WINDOW = "Final Puck Mask Window";
static const std::string PUCK_MARKERS_WINDOW = "Puck Markers Window";
static const std::string PUCK_WITHOUT_MARKERS_WINDOW = "Pucks without Markers Window";
static const std::string BGR_WINDOW = "BGR Model Window";
static const std::string CONTOURS_WINDOW = "Contours Window";

struct ColorParameters {
	// variáveis usadas para o processamento de Black Mask
	int thresh_0;
	int erosion_0;
	// variáveis usadas para o processamento de Pucks Mask
	int thresh_1;
	int close_1;
	int open_1;
	// variáveis usadas para o processamento de Color Mask
	int initial_range_value;
	int range_width;
	// variáveis usadas para o processamento de Final Mask
	int open_2;
	int close_2;
	int open_3;
	// váriaveis usadas para o filtro de áreas aleatórias
	int min_area;
};

struct Object {
	// object position in Cartesian x axis
	double x; 
	// object position in Cartesian y axis
	double y;
	// object color
	Color color;
};

struct LampPosts {
	robotino_vision::LampPost left;
	robotino_vision::LampPost right;
}; // para testes

class RobotinoVision
{

public:

	RobotinoVision();
	~RobotinoVision();

	bool spin();
private:

	ros::NodeHandle nh_;

	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::ServiceServer find_objects_srv_; 
	ros::ServiceServer find_areas_srv_; 
	ros::ServiceServer get_list_srv_;
	ros::ServiceServer contain_in_list_srv_;
	ros::ServiceServer save_srv_;
	ros::ServiceServer set_calibration_srv_;
	ros::ServiceServer get_lamp_posts_srv_;

	cv::Mat imgRGB_;	
	std::vector<LampPosts> lamp_posts_; // para testes
	double randomDouble(double min, double max);
	int randomInteger(int min, int max);

	Color color_;
	bool verify_markers_;
	int specific_number_of_markers_; // in all objects
	std::vector<int> number_of_markers_; // in each object

	double camera_height_;
	double camera_close_distance_;
	double camera_far_distance_;
	double camera_depth_width_;
	int height_;
	int width_;

	int close_aux_, open_aux_, max_area_, dilate_aux_, thresh_area_, close_area_, dilate_area_;
	int pucks_blur_size_, pucks_dilate_, pucks_thresh_, pucks_close_, pucks_open_;
	int color_blur_size_, color_dilate_, color_close_, color_open_;

	bool calibration_;
	std::string contours_window_name_;
	std::string without_markers_window_name_;
	std::string color_name_;

	ColorParameters color_params_, orange_params_, red_params_, green_params_, blue_params_, yellow_params_;

	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	bool findObjects(robotino_vision::FindObjects::Request &req, robotino_vision::FindObjects::Response &res);
	bool findAreas(robotino_vision::FindInsulatingTapeAreas::Request &req, robotino_vision::FindInsulatingTapeAreas::Response &res);
	bool getList(robotino_vision::GetObjectsList::Request &req, robotino_vision::GetObjectsList::Response &res);
	bool getLampPosts(robotino_vision::GetLampPosts::Request &req, robotino_vision::GetLampPosts::Response &res);
	bool containInList(robotino_vision::ContainInList::Request &req, robotino_vision::ContainInList::Response &res);
	bool saveImage(robotino_vision::SaveImage::Request &req, robotino_vision::SaveImage::Response &res);
	bool setCalibration(robotino_vision::SetCalibration::Request &req, robotino_vision::SetCalibration::Response &res);

	bool readImage(std::string image_name);
	std::vector<cv::Point2f> processColor();
	std::vector<cv::Point2f> processColor(Color color);
	std::vector<cv::Point2f> getContours(cv::Mat &input);
	cv::Mat getAllMarkers(cv::Mat &pucks_mask);
	cv::Mat getInsulatingTapeArea();
	cv::Mat getPucksMask();
	cv::Mat getColorMask();
	cv::Mat getPuckMarkers(cv::Mat &black_mask, cv::Mat &pucks_mask);
	cv::Mat getFinalMask(cv::Mat &pucks_mask, cv::Mat &color_mask);
	cv::Mat getFinalMask(cv::Mat &black_mask, cv::Mat &pucks_mask, cv::Mat &color_mask);
	cv::Mat getPuckWithoutMarkers(cv::Mat &final_mask, cv::Mat &puck_markers);
	void showImageBGRwithMask(cv::Mat &mask);
	void setColor();
	void setColor(Color color);
	void setColorParameters();
	std::vector<cv::Point2f> getPositions(std::vector<cv::Point2f> mass_center);
	int getNumberOfObjects();
	int getNumberOfObjects(Color color);
	std::vector<Color> getObjectsInOrder();
	std::vector<Color> getObjectsInOrder(bool from_left_to_right);
	std::vector<Color> orderObjects(std::vector<Object> clutteredObjects, bool from_left_to_right);
	float getClosestObjectDistance(Color color);

	void setImagesWindows();

	void readParameters();

	/*// variaveis usadas para o processamento de LampPost
	void processImageLampPost();
	int threshVal_;
	int dilationSize_;
	int redThreshMinVal1_;
	int redThreshMaxVal1_;
	int redThreshMinVal2_;
	int redThreshMaxVal2_;
	int yellowThreshMinVal_;
	int yellowThreshMaxVal_;
	int greenThreshMinVal_;
	int greenThreshMaxVal_;
*/
};

#endif /* RobotinoVision_H */
