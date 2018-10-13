/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <cstdint>
#include <string>

namespace std
{
template <class T>
class allocator;
}

namespace geometry_msgs
{
template <class ContainerAllocator>
struct Pose_;
typedef Pose_<std::allocator<void>> Pose;
}  // namespace geometry_msgs

namespace sensor_msgs
{
template <class ContainerAllocator>
struct LaserScan_;
typedef LaserScan_<std::allocator<void>> LaserScan;
}  // namespace sensor_msgs

namespace mrpt
{
namespace poses
{
class CPose3D;
}
}  // namespace mrpt
#include <mrpt/version.h>
namespace mrpt
{
namespace obs
{
class CObservation2DRangeScan;
}
}  // namespace mrpt

namespace mrpt_bridge
{
/** @name LaserScan: ROS <-> MRPT
 *  @{ */

/** ROS->MRPT: Takes a sensor_msgs::LaserScan and the relative pose of the laser
 * wrt base_link and builds a CObservation2DRangeScan
 * \return true on sucessful conversion, false on any error.
 * \sa mrpt2ros
 */
bool convert(
	const sensor_msgs::LaserScan& _msg, const mrpt::poses::CPose3D& _pose,
	mrpt::obs::CObservation2DRangeScan& _obj);

/** MRPT->ROS: Takes a CObservation2DRangeScan and outputs range data in
 * sensor_msgs::LaserScan
 * \return true on sucessful conversion, false on any error.
 * \sa ros2mrpt
 */
bool convert(
	const mrpt::obs::CObservation2DRangeScan& _obj,
	sensor_msgs::LaserScan& _msg);

/** MRPT->ROS: Takes a CObservation2DRangeScan and outputs range data in
 * sensor_msgs::LaserScan + the relative pose of the laser wrt base_link
 * \return true on sucessful conversion, false on any error.
 * \sa ros2mrpt
 */
bool convert(
	const mrpt::obs::CObservation2DRangeScan& _obj,
	sensor_msgs::LaserScan& _msg, geometry_msgs::Pose& _pose);

/** @} */

}  // namespace mrpt_bridge
