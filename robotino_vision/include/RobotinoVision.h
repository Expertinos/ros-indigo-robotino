/*
 * RobotinoVision.h
 *
 *  Created on: 15/07/2014
 *      Author: adrianohrl@unifei.edu.br
 */

#ifndef RobotinoVision_H
#define RobotinoVision_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

#include "robotino_vision/FindObjects.h"
#include "robotino_vision/SaveImage.h"
#include "robotino_vision/SetCalibration.h"
#include "robotino_vision/GetProductsList.h"

using namespace cv;
using namespace std;

typedef enum {ORANGE, YELLOW, BLUE, GREEN, RED, BLACK} Color;

static const std::string BLACK_MASK_WINDOW = "Black Mask Window";
static const std::string PUCKS_MASK_WINDOW = "Pucks Mask Window";
static const std::string COLOR_MASK_WINDOW = "Color Mask Window";
static const std::string FINAL_MASK_WINDOW = "Final Mask Window";
static const std::string BGR_WINDOW = "BGR Model Window";
static const std::string CONTOURS_WINDOW = "Contours Window";

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
	ros::ServiceServer save_srv_;
	ros::ServiceServer set_calibration_srv_;
	ros::ServiceServer find_objects_srv_; 
	ros::ServiceServer get_list_srv_;

	Mat imgRGB_;

	Color color_;

	double camera_height_;
	double camera_close_distance_;
	double camera_far_distance_;
	double camera_depth_width_;
	int height_;
	int width_;

	bool calibration_;

	// variáveis usadas para o processamento de Black Mask
	int thresh0_;
	int erosion0_;
	// variáveis usadas para o processamento de Pucks Mask
	int thresh1_;
	int close1_;
	int open1_;
	// variáveis usadas para o processamento de Color Mask
	int initialRangeValue_;
	int rangeWidth_;
	// variáveis usadas para o processamento de Final Mask
	int open2_;
	int close2_;
	int open3_;

	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	bool saveImage(robotino_vision::SaveImage::Request &req, robotino_vision::SaveImage::Response &res);
	bool setCalibration(robotino_vision::SetCalibration::Request &req, robotino_vision::SetCalibration::Response &res);
	bool findObjects(robotino_vision::FindObjects::Request &req, robotino_vision::FindObjects::Response &res);
	bool getList(robotino_vision::GetProductsList::Request &req, robotino_vision::GetProductsList::Response &res);

	cv::Mat readImage(const char* imageName);
	std::vector<cv::Point2f> processColor();
	std::vector<cv::Point2f> getContours(cv::Mat input);
	cv::Mat getBlackMask();
	cv::Mat getPucksMask();
	cv::Mat getColorMask();
	cv::Mat getFinalMask(cv::Mat black_mask, cv::Mat pucks_mask, cv::Mat color_mask);
	void showImageBGRwithMask(cv::Mat mask);
	void setColor(Color color);
	std::vector<cv::Point2f> getPositions(std::vector<cv::Point2f> mass_center);
	int getNumberOfObjects(Color color);

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
