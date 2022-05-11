/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
	APPLICATION: mrpt_ros bridge
	FILE: image.cpp
	AUTHOR: Raghavender Sahdev <raghavendersahdev@gmail.com>
  ---------------------------------------------------------------*/

#include "ros/ros.h"
#include "mrpt_bridge/image.h"
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <mrpt/ros1bridge/time.h>
#include <mrpt/ros1bridge/image.h>

using namespace mrpt::img;
using namespace ros;
using namespace sensor_msgs;
using namespace cv;
using namespace cv_bridge;

bool mrpt_bridge::image::ros2mrpt(
	const sensor_msgs::Image& msg, mrpt::obs::CObservationImage& obj)
{
	obj.timestamp = mrpt::ros1bridge::fromROS(msg.header.stamp);
	obj.image = mrpt::ros1bridge::fromROS(msg);
	return true;
}

bool mrpt_bridge::image::mrpt2ros(
	const mrpt::obs::CObservationImage& obj, const std_msgs::Header& msg_header,
	sensor_msgs::Image& msg)
{
	msg = mrpt::ros1bridge::toROS(obj.image, msg_header);
	return true;
}

//
/*
std_msgs/Header header
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data
 */
