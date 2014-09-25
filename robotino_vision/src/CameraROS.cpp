/*
 * CameraROS.cpp
 *
 *  Created on: 14/07/2014
 *      Author: adrianohrl@unifei.edu.br
 */

#include "CameraROS.h"

CameraROS::CameraROS()
	: enableImageReceivedEvent_(true),
	  isInOrdinaryMode_(false),
	 // counter(0),
	  updatingImgRGB_(false),
	  threshVal(250),
	  dilationSize(2),
	  redThreshMinVal1(0),
	  redThreshMaxVal1(10),
	  redThreshMinVal2(228),
	  redThreshMaxVal2(255),
	  yellowThreshMinVal(15),
	  yellowThreshMaxVal(55),
	  greenThreshMinVal(58),
	  greenThreshMaxVal(96)

{	
	//setFormat(640, 320, "raw");
	imgRGB_ = Mat(320, 240, CV_8UC3, Scalar::all(0));
	activateLampPostMode();
	//setBGREnabled(true);	
}

CameraROS::~CameraROS()
{	
	isInOrdinaryMode_ = false;
	toggleMode();
}

void CameraROS::setNumber(int number)
{
	setCameraNumber(number);
}

void CameraROS::setTimeStamp(ros::Time stamp)
{
	stamp_ = stamp;
}

void CameraROS::imageReceivedEvent(
		const unsigned char* data, 
		unsigned int dataSize,
		unsigned int width, 
		unsigned int height, 
		unsigned int step)
{
	/*if (counter > 100)
	{
		counter = 0;
		toggleMode();
	}
	if (enableImageReceivedEvent_)
	{
		ROS_INFO("Image Format: (height: %d, width: %d)", height, width);
		if (isInOrdinaryMode_)
		{
			ROS_INFO("Running Mode: Ordinary Mode");
			activateOrdinaryMode();
		}
		else
		{
			activateLampPostMode();
			ROS_INFO("Running Mode: Lamp Post Mode");
		}
		imgBGR_ = Mat(height, width, CV_8UC3, (void*) data, step); 
		imshow("Amostragem", imgBGR_);
		waitKey(80);		
	}
	counter++;*/
	activateLampPostMode();
	//setBGREnabled(false);
	updatingImgRGB_ = true;
	imgRGB_ = Mat(height, width, CV_8UC3, (void*) data, step);
	updatingImgRGB_ = false;
	processImage();
}

Mat CameraROS::getImage()
{
	while (updatingImgRGB_) {}
	return imgRGB_;
}

void CameraROS::processImage()
{
	Mat imgBGR;
	cvtColor(imgRGB_, imgBGR, CV_RGB2BGR);
	//imshow("BRG Model", imgBGR);
	Mat imgHSL;
	cvtColor(imgRGB_, imgHSL, CV_RGB2HLS);
	//imshow("HSL Model", imgHSL);
	Mat splittedImgHSL[3];
	split(imgHSL, splittedImgHSL);
	Mat imgH = splittedImgHSL[0];
	//imshow("Hue", imgH);
	Mat imgS = splittedImgHSL[1];
	//imshow("Saturation", imgS);
	Mat imgL = splittedImgHSL[2];
	//imshow("Lightness", imgL);
	
	Mat threshImgL;
	//namedWindow("Lightness Window", 1);
	//createTrackbar("Lightness threshold value:", "Lightness Window", &threshVal, 255);
	int maxVal = 255;
	threshold(imgL, threshImgL, threshVal, maxVal, THRESH_BINARY);
//	std::vector<std::vector<Point>> contours;
//	std::vector<Vec4i> hierarchy;
//	findContours(threshImgL, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
//	Mat imgBorders = Mat::zeros(threshImgL.size(), CV_8U);
//	for (int i = 0; i < contours.size(); i++)
//		drawContours(imgBorders, contours, i, Scalar(255), 2, 8, hierarchy, 0, Point());
//	imshow("Contours", imgBorders);
	//imshow("Lightness Window", threshImgL);
	
	/*const int maskSize = 7;
	int mask[maskSize][maskSize] = {{0, 0, 0, 0, 0, 0, 0},
					{0, 0, 0, 0, 0, 0, 0},
					{0, 0, 0, 0, 0, 0, 0}, 
					{1, 1, 1, 0, 1, 1, 1},
					{0, 0, 0, 0, 0, 0, 0},
					{0, 0, 0, 0, 0, 0, 0}, 
					{0, 0, 0, 0, 0, 0, 0}};  
	Mat kernel = Mat(maskSize, maskSize, CV_8U, (void*) mask, maskSize);*/
	//namedWindow("Dilated Lightness", 1);
	//createTrackbar("Dilation Size", "Dilated Lightness", &dilationSize, 10);
	Mat element = getStructuringElement(MORPH_RECT, Size(2 * dilationSize + 1, 2 * dilationSize + 1), Point(dilationSize, dilationSize)); 
	/*std::sstream == "";
	for (t_size i = 0; i < element.rows; i++)
	{
		for (t_size j = 0; j < element.columns; j++)
			s << 
		s << std::endl;
	}	*/

	Mat dilatedImgL;
	dilate(threshImgL, dilatedImgL, element);
	//imshow("Dilated Lightness", dilatedImgL);
	
	Mat mask = dilatedImgL - threshImgL;
	//imshow("Mask", mask);
	mask /= 255;
/*
	Mat imgH1, imgH2; 
	imgH.copyTo(imgH1);//, CV_8U);
	Scalar s1 = Scalar(25);
	imgH1 += s1;
	//std::cout << imgH1;
	imgH.copyTo(imgH2);//, CV_8U;
	Scalar s2 = Scalar(230);
	imgH2 -= s2;
	Mat newImgH = imgH1 + imgH2;*/
	vector<Mat> masks;
	masks.push_back(mask);
	masks.push_back(mask);
	masks.push_back(mask);
	Mat imgMask;
	merge(masks, imgMask);
	Mat maskedImgBGR;
	maskedImgBGR = imgBGR.mul(imgMask);
	Mat finalImgH = imgH.mul(mask); 
	//imshow("Final Filter", maskedImgBGR);

	Mat imgRed, imgRed1, imgRed2, imgRed3, imgRed4;
	//int redThreshMinVal1 = 0;
	//int redThreshMaxVal1 = 10;
	//namedWindow("Red Range", 1);
	//createTrackbar("Range1MinVal", "Red Range", &redThreshMinVal1, 255);
	//createTrackbar("Range1MaxVal", "Red Range", &redThreshMaxVal1, 255);
	int redMaxVal = 1;
	threshold(finalImgH, imgRed1, redThreshMinVal1, redMaxVal, THRESH_BINARY);
	threshold(finalImgH, imgRed2, redThreshMaxVal1, redMaxVal, THRESH_BINARY_INV);
	imgRed3 = imgRed1.mul(imgRed2); 
	//int redThreshMinVal2 = 228;
	//int redThreshMaxVal2 = 255;
	//createTrackbar("Range2MinVal", "Red Range", &redThreshMinVal2, 255);
	//createTrackbar("Range2MaxVal", "Red Range", &redThreshMaxVal2, 255);
	threshold(finalImgH, imgRed1, redThreshMinVal2, redMaxVal, THRESH_BINARY);
	threshold(finalImgH, imgRed2, redThreshMaxVal2, redMaxVal, THRESH_BINARY_INV);
	imgRed4 = imgRed1.mul(imgRed2);
	imgRed = (imgRed3 + imgRed4) * 255;
	//imshow("Red Range", imgRed);

	Mat imgYellow, imgYellow1, imgYellow2;
	//int yellowThreshMinVal = 15;
	//int yellowThreshMaxVal = 55;
	//namedWindow("Yellow Range", 1);
	//createTrackbar("RangeMinVal", "Yellow Range", &yellowThreshMinVal, 255);
	//createTrackbar("RangeMaxVal", "Yellow Range", &yellowThreshMaxVal, 255);
	int yellowMaxVal = 1;
	threshold(finalImgH, imgYellow1, yellowThreshMinVal, yellowMaxVal, THRESH_BINARY);
	threshold(finalImgH, imgYellow2, yellowThreshMaxVal, yellowMaxVal, THRESH_BINARY_INV);
	imgYellow = imgYellow1.mul(imgYellow2) * 255;
	//imshow("Yellow Range", imgYellow);
	
	Mat imgGreen, imgGreen1, imgGreen2;
	//int greenThreshMinVal = 58;
	//int greenThreshMaxVal = 96;
	//namedWindow("Green Range", 1);
	//createTrackbar("RangeMinVal", "Green Range", &greenThreshMinVal, 255);
	//createTrackbar("RangeMaxVal", "Green Range", &greenThreshMaxVal, 255);
	int greenMaxVal = 1;
	threshold(finalImgH, imgGreen1, greenThreshMinVal, greenMaxVal, THRESH_BINARY);
	threshold(finalImgH, imgGreen2, greenThreshMaxVal, greenMaxVal, THRESH_BINARY_INV);
	imgGreen = imgGreen1.mul(imgGreen2) * 255;
	//imshow("Green Range", imgGreen);

	
	waitKey(80);
}

void CameraROS::setEnableImageReceivedEvent(bool enable)
{	
	enableImageReceivedEvent_ = enable;
}

void CameraROS::toggleMode()
{
	if (isInOrdinaryMode_)
		activateLampPostMode();
	else
		activateOrdinaryMode();
	//setBGREnabled(true);
}

void CameraROS::activateLampPostMode()
{	
	isInOrdinaryMode_ = false;
	setAutoFocusEnabled(false);
	setFocus(3000);
	//setAutoExposureEnabled(false);
	//setExposure(50);
	//setGain(5);
}

void CameraROS::activateOrdinaryMode()
{
	isInOrdinaryMode_ = true;
	setAutoFocusEnabled(true);
	setAutoExposureEnabled(true);
	setAutoWhiteBalanceEnabled(true);
	/*const char* usbPortPath = "/dev/bus/usb/001/019";
	if (resetCameraUSBPort(usbPortPath))
		ROS_INFO("The %s has not been resetted successfully!!!", usbPortPath);
	else
		ROS_INFO("The %s has been resetted successfully!!!", usbPortPath);*/
}

/*bool CameraROS::resetCameraUSBPort(const char* usbPortPath)
{
	bool succeed = false;
	int fd, rc;
	fd = open(usbPortPath, O_WRONLY);	
	rc = ioctl(fd, USBDEVFS_RESET, 0);
	close(fd);
	if (rc >= 0)
		succeed = true;
	return succeed;
}*/
