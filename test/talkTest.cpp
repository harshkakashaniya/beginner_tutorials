/************************************************************************
 MIT License

 Copyright (c) 2018 Harsh Kakashaniya

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 *************************************************************************/

/**
 *  @file    talkTest.cpp
 *  @author  Harsh Kakashaniya
 *  @date    13/04/2018
 *  @version 1.0
 *
 *  @brief UMD ENPM 808X, ROS tutorials.
 *
 *  @Description DESCRIPTION
 *
 *  This file is used to include all test services.
 *
 */

 // Call ROS library for using its functions
#include "ros/ros.h"
// Bring in gtest
#include <gtest/gtest.h>

#include "beginner_tutorials/change_string.h"

/**
 * @brief Testing is service working
 *
 * @param TalkerService (Test group name)
 * @param testInitOfService(name)
*/

TEST(TalkerService, testInitOfService) {
  ros::NodeHandle nh;
  ros::ServiceClient client =
      nh.serviceClient<beginner_tutorials::change_string>("change_string");
  // Check if the client exists
  bool ServiceStatus(client.waitForExistence(ros::Duration(3.0)));
  EXPECT_TRUE(ServiceStatus);
}
