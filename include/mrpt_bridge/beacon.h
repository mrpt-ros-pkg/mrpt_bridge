/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <stdint.h>
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

namespace mrpt_msgs
{
template <class ContainerAllocator>
struct ObservationRangeBeacon_;
typedef ObservationRangeBeacon_<std::allocator<void>> ObservationRangeBeacon;
}  // namespace mrpt_msgs

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
class CObservationBeaconRanges;
}
}  // namespace mrpt

namespace mrpt_bridge
{
/** @name ObservationRangeBeacon: ROS <-> MRPT
 *  @{ */

/** ROS->MRPT: Takes a mrpt_msgs::ObservationRangeBeacon and the relative pose
 * of the range sensor wrt base_link and builds a CObservationBeaconRanges
 * \return true on sucessful conversion, false on any error.
 * \sa mrpt2ros
 */
bool convert(
	const mrpt_msgs::ObservationRangeBeacon& _msg,
	const mrpt::poses::CPose3D& _pose,

	mrpt::obs::CObservationBeaconRanges& _obj);

/** MRPT->ROS: Takes a CObservationBeaconRanges and outputs range data in
 * mrpt_msgs::ObservationRangeBeacon
 * \return true on sucessful conversion, false on any error.
 * \sa ros2mrpt
 */
bool convert(

	const mrpt::obs::CObservationBeaconRanges& _obj,
	mrpt_msgs::ObservationRangeBeacon& _msg);

/** MRPT->ROS: Takes a CObservationBeaconRanges and outputs range data in
 * mrpt_msgs::ObservationRangeBeacon + the relative pose of the range sensor wrt
 * base_link
 * \return true on sucessful conversion, false on any error.
 * \sa ros2mrpt
 */
bool convert(
	const mrpt::obs::CObservationBeaconRanges& _obj,
	mrpt_msgs::ObservationRangeBeacon& _msg, geometry_msgs::Pose& _pose);

/** @} */

}  // namespace mrpt_bridge
