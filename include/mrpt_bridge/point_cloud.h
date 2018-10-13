/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <sensor_msgs/PointCloud.h>

#include <mrpt/version.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CColouredPointsMap.h>

namespace mrpt_bridge
{
/** @name Point clouds: ROS <-> MRPT
 *  @{ */

/** Methods to convert between ROS msgs and MRPT objects for point-cloud
 * datatypes.
 */
namespace point_cloud
{
/** Convert sensor_msgs/PointCloud -> mrpt::maps::CSimplePointsMap
 *  CSimplePointsMap only contains (x,y,z) data, so
 * sensor_msgs::PointCloud::channels are ignored.
 * \return true on sucessful conversion, false on any error.
 * \sa mrpt2ros
 */
bool ros2mrpt(
	const sensor_msgs::PointCloud& msg, mrpt::maps::CSimplePointsMap& obj);

/** Convert mrpt::maps::CSimplePointsMap -> sensor_msgs/PointCloud
 *  The user must supply the "msg_header" field to be copied into the output
 * message object, since that part does not appear in MRPT classes.
 *
 *  Since CSimplePointsMap only contains (x,y,z) data,
 * sensor_msgs::PointCloud::channels will be empty.
 * \return true on sucessful conversion, false on any error.
 * \sa ros2mrpt
 */
bool mrpt2ros(
	const mrpt::maps::CSimplePointsMap& obj, const std_msgs::Header& msg_header,
	sensor_msgs::PointCloud& msg);
}  // namespace point_cloud

/** @} */

}  // namespace mrpt_bridge
