/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "mrpt_bridge/time.h"
#include <cmath>  // std::fmod

void mrpt_bridge::convert(const ros::Time& src, mrpt::system::TTimeStamp& des)
{
	// Use the signature of time_tToTimestamp() that accepts "double" with the
	// fractional parts of seconds:
	des = mrpt::system::time_tToTimestamp(src.sec + src.nsec * 1e-9);
}

void mrpt_bridge::convert(const mrpt::system::TTimeStamp& src, ros::Time& des)
{
	// Convert to "double-version of time_t", then extract integer and
	// fractional parts:
	const double t = mrpt::system::timestampTotime_t(src);
	des.sec = static_cast<uint64_t>(t);
	des.nsec = static_cast<uint64_t>(std::fmod(t, 1.0) * 1e9 + 0.5 /*round*/);
}
