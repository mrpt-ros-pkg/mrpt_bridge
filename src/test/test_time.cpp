/*
 * test_time.cpp
 *
 *  Created on: July 15, 2014
 *      Author: Markus Bader
 */

#include <gtest/gtest.h>
#include <mrpt/ros1bridge/time.h>
#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#include <boost/date_time/posix_time/time_formatters.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

TEST(Time, basicTest)
{
	const mrpt::system::TTimeStamp mtime = mrpt::system::getCurrentTime();
	const ros::Time rtimeDes = mrpt::ros1bridge::toROS(mtime);
	const mrpt::system::TTimeStamp mtimeDes =
		mrpt::ros1bridge::fromROS(rtimeDes);

	std::cout << "TimeNow: "
			  << boost::posix_time::to_simple_string(rtimeDes.toBoost())
			  << std::endl;
	EXPECT_EQ(mtime, mtimeDes);
}
