/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/system/datetime.h>
#include <ros/time.h>

namespace mrpt_bridge
{
/**
 * converts ros time to mrpt time
 * @param src ros time
 * @param des mrpt time
 */
void convert(const ros::Time& src, mrpt::system::TTimeStamp& des);

/**
 * converts mrpt time to ros time
 * @param src ros time
 * @param des mrpt time
 */
void convert(const mrpt::system::TTimeStamp& src, ros::Time& des);

};  // namespace mrpt_bridge
