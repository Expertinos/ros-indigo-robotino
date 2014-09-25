/*
 * KinectROS.h
 *
 *  Created on: 20.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#ifndef KINECTROS_H_
#define KINECTROS_H_

#include "rec/robotino/api2/Kinect.h"

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Geometry>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class KinectROS: public rec::robotino::api2::Kinect
{
public:
	KinectROS();
	~KinectROS();

	void setDownsample( bool downsample );
	void setLeafSize( double leaf_size );
	void setTimeStamp(ros::Time stamp);

private:
	ros::NodeHandle nh_;

	ros::Publisher cloud_pub_;

	image_transport::ImageTransport img_transport_;
	image_transport::CameraPublisher streaming_pub_;

	sensor_msgs::Image img_msg_;
	sensor_msgs::CameraInfo cam_info_msg_;

	ros::Time stamp_;

	bool downsample_;
	double leaf_size_;

	float gamma_[2048];
	Eigen::Matrix4f transformation_;

	void init();

	void depthEvent(
			const unsigned short* data,
			unsigned int dataSize,
			unsigned int width,
			unsigned int height,
			unsigned int format,
			unsigned int stamp );

	void videoEvent(
			const unsigned char* data,
			unsigned int dataSize,
			unsigned int width,
			unsigned int height,
			unsigned int step,
			unsigned int format,
			unsigned int stamp );

};


#endif /* KINECTROS_H_ */
