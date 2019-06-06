/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <cstring>  // size_t

#include <mrpt/version.h>
#if MRPT_VERSION < 0x199
#include <mrpt/math/CMatrixFixedNumeric.h>
#else
#include <mrpt/math/CMatrixFixed.h>
#endif

namespace std
{
template <class T>
class allocator;
}

namespace tf
{
class Transform;
class Matrix3x3;
}  // namespace tf

namespace geometry_msgs
{
template <class ContainerAllocator>
struct Pose_;
typedef Pose_<std::allocator<void>> Pose;
template <class ContainerAllocator>
struct PoseWithCovariance_;
typedef PoseWithCovariance_<std::allocator<void>> PoseWithCovariance;
template <class ContainerAllocator>
struct Quaternion_;
typedef Quaternion_<std::allocator<void>> Quaternion;
}  // namespace geometry_msgs

namespace mrpt
{
namespace math
{
template <class T>
class CQuaternion;
struct TPose3D;
struct TPose2D;
}  // namespace math
namespace poses
{
class CPose2D;
class CPose3D;
class CPosePDFGaussian;
class CPosePDFGaussianInf;
class CPose3DPDFGaussian;
class CPose3DPDFGaussianInf;
typedef math::CQuaternion<double> CQuaternionDouble;
}  // namespace poses
}  // namespace mrpt

namespace mrpt_bridge
{
/**\name MRPT  \rightarrow ROS conversions */
/**\{ */

/** Convert: [MRPT] Matrix \rightarrow [ROS] Matrix */
tf::Matrix3x3& convert(
	const mrpt::math::CMatrixDouble33& _src, tf::Matrix3x3& _des);

/** Convert: [MRPT] CPose3D \rightarrow [ROS] Transform */
tf::Transform& convert(const mrpt::poses::CPose3D& _src, tf::Transform& _des);

/** Convert: [MRPT] TPose3D \rightarrow [ROS] Transform */
tf::Transform& convert(const mrpt::math::TPose3D& _src, tf::Transform& _des);
/** Convert: [MRPT] CPose2D \rightarrow [ROS] Transform */
tf::Transform& convert(const mrpt::poses::CPose2D& _src, tf::Transform& _des);
/** Convert: [MRPT] TPose2D \rightarrow [ROS] Transform */
tf::Transform& convert(const mrpt::math::TPose2D& _src, tf::Transform& _des);

/** Convert: [MRPT] CPose3D \rightarrow [ROS] Pose */
geometry_msgs::Pose& convert(
	const mrpt::poses::CPose3D& _src, geometry_msgs::Pose& _des);

/** Convert: [MRPT] TPose2D \rightarrow [ROS] Pose */
geometry_msgs::Pose& convert(
	const mrpt::math::TPose2D& _src, geometry_msgs::Pose& _des);

/** Convert: [MRPT] CPose2D (x,y,yaw) \rightarrow [ROS] Pose */
geometry_msgs::Pose& convert(
	const mrpt::poses::CPose2D& _src, geometry_msgs::Pose& _des);

/** Convert: [MRPT] CPose3DPDFGaussian \rightarrow [ROS] PoseWithCovariance */
geometry_msgs::PoseWithCovariance& convert(
	const mrpt::poses::CPose3DPDFGaussian& _src,
	geometry_msgs::PoseWithCovariance& _des);

/** Convert: [MRPT] CPose3DPDFGaussianInf \rightarrow [ROS] PoseWithCovariance
 */
geometry_msgs::PoseWithCovariance& convert(
	const mrpt::poses::CPose3DPDFGaussianInf& _src,
	geometry_msgs::PoseWithCovariance& _des);

/** Convert: [MRPT] CPose3DPDFGaussian \rightarrow [ROS] Transform */
tf::Transform& convert(
	const mrpt::poses::CPose3DPDFGaussian& _src, tf::Transform& _des);

/** Convert: [MRPT] CPosePDFGaussian (x,y,yaw) \rightarrow [ROS]
 * PoseWithCovariance */
geometry_msgs::PoseWithCovariance& convert(
	const mrpt::poses::CPosePDFGaussian& _src,
	geometry_msgs::PoseWithCovariance& _des);

/** Convert: [MRPT] CPosePDFGaussianInf (x,y,yaw) \rightarrow [ROS]
 * PoseWithCovariance */
geometry_msgs::PoseWithCovariance& convert(
	const mrpt::poses::CPosePDFGaussianInf& _src,
	geometry_msgs::PoseWithCovariance& _des);

/** Convert: [MRPT] CQuaternionDouble \rightarrow [ROS] Quaternion  */
geometry_msgs::Quaternion& convert(
	const mrpt::poses::CQuaternionDouble& _src,
	geometry_msgs::Quaternion& _des);

/**\} */

/**\name ROS \rightarrow MRPT conversions */
/**\{ */

/** Convert: [ROS] CPose3D \rightarrow [MRPT] Transform */
mrpt::poses::CPose3D& convert(
	const tf::Transform& _src, mrpt::poses::CPose3D& _des);

/** Convert: [ROS] Matrix \rightarrow [MRPT] Matrix */
mrpt::math::CMatrixDouble33& convert(
	const tf::Matrix3x3& _src, mrpt::math::CMatrixDouble33& _des);

/** Convert: [ROS] Pose \rightarrow [MRPT] CPose2D  */
mrpt::poses::CPose2D& convert(
	const geometry_msgs::Pose& _src, mrpt::poses::CPose2D& _des);

/** Convert: [ROS] Pose \rightarrow [MRPT] CPose3D  */
mrpt::poses::CPose3D& convert(
	const geometry_msgs::Pose& _src, mrpt::poses::CPose3D& _des);

/** Convert: [ROS] PoseWithCovariance \rightarrow [MRPT] CPose3DPDFGaussian */
mrpt::poses::CPose3DPDFGaussian& convert(
	const geometry_msgs::PoseWithCovariance& _src,
	mrpt::poses::CPose3DPDFGaussian& _des);

/** Convert: [ROS] PoseWithCovariance \rightarrow [MRPT] CPose3DPDFGaussianInf
 */
mrpt::poses::CPose3DPDFGaussianInf& convert(
	const geometry_msgs::PoseWithCovariance& _src,
	mrpt::poses::CPose3DPDFGaussianInf& _des);

/** Convert: [ROS] PoseWithCovariance \rightarrow [MRPT] CPosePDFGaussian
 * (x,y,yaw) */
mrpt::poses::CPosePDFGaussian& convert(
	const geometry_msgs::PoseWithCovariance& _src,
	mrpt::poses::CPosePDFGaussian& _des);

/** Convert: [ROS] PoseWithCovariance \rightarrow [MRPT] CPosePDFGaussianInf
 * (x,y,yaw) */
mrpt::poses::CPosePDFGaussianInf& convert(
	const geometry_msgs::PoseWithCovariance& _src,
	mrpt::poses::CPosePDFGaussianInf& _des);

/** Convert: [ROS] Quaternion \rightarrow [MRPT] CQuaternionDouble  */
mrpt::poses::CQuaternionDouble& convert(
	const geometry_msgs::Quaternion& _src,
	mrpt::poses::CQuaternionDouble& _des);

/**\} */
}  // namespace mrpt_bridge
