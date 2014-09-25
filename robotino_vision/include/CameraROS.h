/*
 * CameraROS.h
 *
 *  Created on: 08.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#ifndef CAMERAROS_H_
#define CAMERAROS_H_

#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//#include <stdio.h>
//#include <unistd.h>
//#include <fcntl.h>
//#include <errno.h>
//#include <sys/ioctl.h>
//#include <linux/usbdevice_fs.h>

#include "rec/robotino/api2/Camera.h"
using namespace cv;

class CameraROS : public rec::robotino::api2::Camera
{
public:
	CameraROS();
	~CameraROS();

	void setNumber(int number);
	void setTimeStamp(ros::Time stamp);
	void setEnableImageReceivedEvent(bool enable);
	void toggleMode();
	Mat getImage();
	void processImage();

private:

	ros::Time stamp_;
	bool enableImageReceivedEvent_;
	bool isInOrdinaryMode_;
	bool updatingImgRGB_;
	Mat imgRGB_;
	//unsigned int counter_;
	int threshVal;
	int dilationSize;
	int redThreshMinVal1;
	int redThreshMaxVal1;
	int redThreshMinVal2;
	int redThreshMaxVal2;
	int yellowThreshMinVal;
	int yellowThreshMaxVal;
	int greenThreshMinVal;
	int greenThreshMaxVal;

	void imageReceivedEvent(
			const unsigned char* data,
			unsigned int dataSize,
			unsigned int width,
			unsigned int height,
			unsigned int step );		
	void activateOrdinaryMode();
	void activateLampPostMode();
	//bool resetCameraUSBPort(const char* usbPortPath);
};

#endif /* CAMERAROS_H_ */
