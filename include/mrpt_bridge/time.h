#ifndef MRPT_BRIDGE_TIME_H
#define MRPT_BRIDGE_TIME_H

#include <mrpt/system/datetime.h>
#include <ros/time.h>
#include <mrpt/version.h>

namespace mrpt_bridge
{
/**
  * converts ros time to mrpt time
  * @param src ros time
  * @param des mrpt time
  */
inline void convert(const ros::Time& src, mrpt::system::TTimeStamp& des)
{
	// return (((uint64_t)src.sec) * (uint64_t)10000000) +
	// ((uint64_t)116444736*1000000000);
#if MRPT_VERSION>=0x199
	des = mrpt::system::time_tToTimestamp((double)src.sec + ((double)src.nsec / 1000000000.d));
#else
	des = mrpt::system::time_tToTimestamp((time_t)src.sec) + src.nsec / 100;
#endif
	// printf("nsec    %" PRIu64 "\n", des);
}

/**
  * converts mrpt time to ros time
  * @param src ros time
  * @param des mrpt time
  */
inline void convert(const mrpt::system::TTimeStamp& src, ros::Time& des)
{
#if MRPT_VERSION>=0x199
	des.sec = (uint64_t)mrpt::system::timestampTotime_t(src);
	des.nsec = (uint64_t)(mrpt::system::timestampTotime_t(src) * 1000000000);
#else
	des.sec = ((uint64_t)src) / ((uint64_t)10000000) -
			  (((uint64_t)116444736) * ((uint64_t)100));
	des.nsec = ((uint64_t)src) % ((uint64_t)10000000) * 100;
#endif
	// printf("ros.sec       %" PRIu32 "\n", des.sec);
	// printf("ros.nsec      %" PRIu32 "\n", des.nsec);
}

};  // namespace mrpt_bridge

#endif
