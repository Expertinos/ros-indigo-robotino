/*
 * KinectROS.cpp
 *
 *  Created on: 20.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#include "KinectROS.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/fill_image.h>

namespace sensor_msgs
{
extern bool fillImage(Image &image, const std::string &encoding_arg, uint32_t rows_arg, uint32_t cols_arg, uint32_t step_arg, const void *data_arg);
};

KinectROS::KinectROS():
	img_transport_(nh_),
	downsample_(true)
{
	cloud_pub_ = nh_.advertise<PointCloud>("kinect", 1 );
	streaming_pub_ = img_transport_.advertiseCamera("image_raw_kinect", 1, false);
	init();
}

KinectROS::~KinectROS()
{
	cloud_pub_.shutdown();
	streaming_pub_.shutdown();
}

void KinectROS::setDownsample( bool downsample )
{
	downsample_ = downsample;
}

void KinectROS::setLeafSize( double leaf_size )
{
	leaf_size_ = leaf_size;
}

void KinectROS::setTimeStamp(ros::Time stamp)
{
	stamp_ = stamp;
}

void KinectROS::init()
{
	for (size_t i=0; i < 2048; i++)
	{
		const float k1 = 1.1863;
		const float k2 = 2842.5;
		const float k3 = 0.1236;
		gamma_[i] = k3 * tan(i/k2 + k1);
	}

	Eigen::Matrix4f transformationX; // Rotate 90 Degrees along X
	transformationX <<
			1, 0, 0, 0,
			0, 0, 1, 0,
			0, -1, 0, 0,
			0, 0, 0, 1;
	Eigen::Matrix4f transformationY; // Rotate 90 Degrees along Z
	transformationY <<
			0, 0, 1, 0,
			0, 1, 0, 0,
			-1, 0, 0, 0,
			0, 0, 0, 1;

	transformation_ = transformationX * transformationY;
}


void KinectROS::depthEvent(
		const unsigned short* data,
		unsigned int dataSize,
		unsigned int width,
		unsigned int height,
		unsigned int format,
		unsigned int stamp )
{

	PointCloud::Ptr msg ( new PointCloud );
	PointCloud msg_transformed;

	// Build the cloud message
	msg->header.stamp = stamp_;
	msg->header.frame_id = "kinect_link";
	msg->height = height;
	msg->width = width;
	msg->is_dense = false;

	msg->points.resize( msg->height * msg->width );

	// camera intrinsic parameters, representative values, see http://nicolas.burrus.name/index.php/Research/KinectCalibration for more info
	float cx = 320.0; // center of projection
	float cy = 240.0; // center of projection
	float fx = 600.0; // focal length in pixels
	float fy = 600.0; // focal length in pixels
	for (size_t v=0, n=0 ; v<480 ; v++)
	{
		for (size_t u=0 ; u<640 ; u++, n++)
		{
			// note that values will be in meters, with the camera at the origin, and the Kinect has a minimum range of ~0.5 meters
			msg->points[n].x = (u - cx) * gamma_[data[n]] / fx;
			msg->points[n].y = (v - cy) * gamma_[data[n]] / fy;
			msg->points[n].z = gamma_[data[n]];
			if( msg->points[n].z < 0.2 ) //to close
			{
				msg->points[n].x = msg->points[n].y = msg->points[n].z =
						std::numeric_limits<float>::quiet_NaN();
			}
		}
	}

	pcl::transformPointCloud (*msg, msg_transformed, transformation_ );

	if( downsample_ )
	{
		sensor_msgs::PointCloud2 cloud;
		sensor_msgs::PointCloud2::Ptr cloud_downsampled (new sensor_msgs::PointCloud2());
		pcl::VoxelGrid<sensor_msgs::PointCloud2> vg;
		pcl::toROSMsg( msg_transformed, cloud );
		vg.setInputCloud( boost::make_shared<sensor_msgs::PointCloud2> (cloud) );
		vg.setLeafSize (leaf_size_, leaf_size_, leaf_size_);
		vg.filter (*cloud_downsampled);

		cloud_pub_.publish( cloud_downsampled );
	}
	else
	{
		cloud_pub_.publish( msg );
	}
}

void KinectROS::videoEvent(
		const unsigned char* data,
		unsigned int dataSize,
		unsigned int width,
		unsigned int height,
		unsigned int step,
		unsigned int format,
		unsigned int stamp )
{
	// Build the Image msg
	img_msg_.header.stamp = stamp_;
	sensor_msgs::fillImage(img_msg_, "bgr8", height, width, step, data);

	// Build the CameraInfo msg
	cam_info_msg_.header.stamp = ros::Time::now();
	cam_info_msg_.height = height;
	cam_info_msg_.width = width;

	// Publish the Image & CameraInfo msgs
	streaming_pub_.publish(img_msg_, cam_info_msg_);
}
