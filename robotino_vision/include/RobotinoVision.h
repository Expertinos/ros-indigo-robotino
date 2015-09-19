/*
 * RobotinoVision.h
 *
 *  Created on: 15/07/2014
 *      Author: adrianohrl@unifei.edu.br
 */

#ifndef RobotinoVision_H
#define RobotinoVision_H

#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <fl/Headers.h>

#include "robotino_vision/FindObjects.h"
#include "robotino_vision/SaveImage.h"
#include "robotino_vision/SetCalibration.h"
#include "robotino_vision/GetProductsList.h"

typedef enum {ORANGE, YELLOW, BLUE, GREEN, RED, BLACK} Color;

static const std::string BLACK_MASK_WINDOW = "Black Mask Window";
static const std::string PUCKS_MASK_WINDOW = "Pucks Mask Window";
static const std::string COLOR_MASK_WINDOW = "Color Mask Window";
static const std::string FINAL_MASK_WINDOW = "Final Mask Window";
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
};

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

	cv::Mat imgRGB_;

	Color color_;

	double camera_height_;
	double camera_close_distance_;
	double camera_far_distance_;
	double camera_depth_width_;
	int height_;
	int width_;

	bool calibration_;

	ColorParameters color_params_, orange_params_, red_params_, green_params_, blue_params_, yellow_params_;

	/*// variáveis usadas para o processamento de Black Mask
	int thresh0_, orange_thresh0_, red_thresh0_, green_thresh0_, blue_thresh0_, yellow_thresh0_;
	int erosion0_, orange_erosion0_, red_erosion0_, green_erosion0_, blue_erosion0_, yellow_erosion0_;
	// variáveis usadas para o processamento de Pucks Mask
	int thresh1_, orange_thresh1_, red_thresh1_, green_thresh1_, blue_thresh1_, yellow_thresh1_;
	int close1_, orange_close1_, red_close1_, green_close1_, blue_close1_, yellow_close1_;
	int open1_, orange_open1_, red_open1_, green_open1_, blue_open1_, yellow_open1_;
	// variáveis usadas para o processamento de Color Mask
	int initial_range_value_, orange_initial_range_value_, red_initial_range_value_, green_initial_range_value_, blue_initial_range_value_, yellow_initial_range_value_;
	int range_width_, orange_range_width_, red_range_width_, green_range_width_, blue_range_width_, yellow_range_width_;
	// variáveis usadas para o processamento de Final Mask
	int open2_, orange_open2_, red_open2_, green_open2_, blue_open2_, yellow_open2_;
	int close2_, orange_close2_, red_close2_, green_close2_, blue_close2_, yellow_close2_;
	int open3_, orange_open3_, red_open3_, green_open3_, blue_open3_, yellow_open3_;*/

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
