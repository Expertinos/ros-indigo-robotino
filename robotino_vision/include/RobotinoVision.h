/*
 * RobotinoVision.h
 *
 *  Created on: 15/07/2014
 *      Author: adrianohrl@unifei.edu.br
 */

#ifndef RobotinoVision_H
#define RobotinoVision_H

//#include "robotino_vision/LampPostState.h"
//#include "robotino_vision/PuckState.h"
//#include "FindPuck.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "robotino_vision/SaveImage.h"

using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

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

	void processImageLampPost();
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	void processImagePucks(unsigned char initialRangeValue, unsigned char rangeWidth);
	Mat readImage(char* imageName);
	bool saveImage(robotino_vision::SaveImage::Request &req, robotino_vision::SaveImage::Response &res);

	bool enableImageReceivedEvent_;
	bool isInOrdinaryMode_;
	bool updatingImgRGB_;
	Mat imgRGB_;

	// variaveis usadas para o processamento de LampPost
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

	// variáveis usadas para o processamento de Pucks
	int threshV_;
	int erosionSize_;
	int threshS_;
	int closeSize1_;
	int openSize1_;

	
};

#endif /* RobotinoVision_H */
