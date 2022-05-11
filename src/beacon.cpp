/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <geometry_msgs/Pose.h>
#include <mrpt/ros1bridge/time.h>
#include <mrpt/ros1bridge/pose.h>
#include <mrpt_msgs/ObservationRangeBeacon.h>
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "mrpt_bridge/beacon.h"
#include <mrpt/obs/CObservationBeaconRanges.h>

using namespace mrpt::obs;

namespace mrpt_bridge
{
bool convert(
	const mrpt_msgs::ObservationRangeBeacon& _msg,
	const mrpt::poses::CPose3D& _pose, CObservationBeaconRanges& _obj)
{
	_obj.timestamp = mrpt::ros1bridge::fromROS(_msg.header.stamp);
	mrpt::poses::CPose3D cpose_obj;

	_obj.stdError = _msg.sensor_std_range;
	_obj.sensorLabel = _msg.header.frame_id;
	_obj.maxSensorDistance = _msg.max_sensor_distance;
	_obj.minSensorDistance = _msg.min_sensor_distance;

	if (_pose.empty())
	{
		cpose_obj = mrpt::ros1bridge::fromROS(_msg.sensor_pose_on_robot);
		_obj.setSensorPose(cpose_obj);
	}
	else
	{
		_obj.setSensorPose(_pose);
	}

	ASSERT_(_msg.sensed_data.size() >= 1);
	const size_t N = _msg.sensed_data.size();

	_obj.sensedData.resize(N);

	for (std::size_t i_mrpt = 0; i_mrpt < N; i_mrpt++)
	{
		_obj.sensedData[i_mrpt].sensedDistance = _msg.sensed_data[i_mrpt].range;
		_obj.sensedData[i_mrpt].beaconID = _msg.sensed_data[i_mrpt].id;
	}
	return true;
}

bool convert(
	const CObservationBeaconRanges& _obj,
	mrpt_msgs::ObservationRangeBeacon& _msg)
{
	mrpt::poses::CPose3D cpose_obj;

	_msg.header.stamp = mrpt::ros1bridge::toROS(_obj.timestamp);
	_obj.getSensorPose(cpose_obj);
	_msg.sensor_pose_on_robot = mrpt::ros1bridge::toROS_Pose(cpose_obj);

	_msg.sensor_std_range = _obj.stdError;
	_msg.header.frame_id = _obj.sensorLabel;
	_msg.max_sensor_distance = _obj.maxSensorDistance;
	_msg.min_sensor_distance = _obj.minSensorDistance;

	ASSERT_(_obj.sensedData.size() >= 1);
	const size_t N = _obj.sensedData.size();

	_msg.sensed_data.resize(N);

	for (std::size_t i_msg = 0; i_msg < N; i_msg++)
	{
		_msg.sensed_data[i_msg].range = _obj.sensedData[i_msg].sensedDistance;
		_msg.sensed_data[i_msg].id = _obj.sensedData[i_msg].beaconID;
	}
	return true;
}

bool convert(
	const CObservationBeaconRanges& _obj,
	mrpt_msgs::ObservationRangeBeacon& _msg, geometry_msgs::Pose& _pose)
{
	convert(_obj, _msg);
	mrpt::poses::CPose3D pose;
	_obj.getSensorPose(pose);
	_pose = mrpt::ros1bridge::toROS_Pose(pose);
	return true;
}
}  // namespace mrpt_bridge
