#ifndef MRPT_BRIDGE_TIME_H
#define MRPT_BRIDGE_TIME_H

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

#endif
