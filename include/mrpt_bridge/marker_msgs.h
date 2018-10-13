/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

/**
 * @file   marker_msgs.h
 * @author Markus Bader <markus.bader@tuwien.ac.at>
 * @date   September, 2017
 * @brief  Funtions to convert marker_msgs to mrpt msgs
 **/

#pragma once

#include <marker_msgs/MarkerDetection.h>
#include <mrpt/obs/CObservationRange.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/obs/CObservationBeaconRanges.h>

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

namespace marker_msgs
{
template <class ContainerAllocator>
struct MarkerDetection_;
typedef MarkerDetection_<std::allocator<void>> MarkerDetection;
}  // namespace marker_msgs

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
class CObservationBearingRange;
}
}  // namespace mrpt

namespace mrpt_bridge
{
bool convert(
	const marker_msgs::MarkerDetection& _msg, const mrpt::poses::CPose3D& _pose,
	mrpt::obs::CObservationBearingRange& _obj);
bool convert(
	const marker_msgs::MarkerDetection& _msg, const mrpt::poses::CPose3D& _pose,
	mrpt::obs::CObservationBeaconRanges& _obj);
}  // namespace mrpt_bridge
