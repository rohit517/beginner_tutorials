/*
 MIT License

 Copyright (c) 2018 Rohit

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
 */

/**
 *  @file    talkerTest.cpp
 *  @author  Rohitkrishna Nambiar (rohit517)
 *  @date    11/13/2018
 *  @version 1.0
 *
 *  @brief Talker node rostest
 *
 *  @section DESCRIPTION
 *
 *  Test file to run tests for talker node.
 *
 */

// Gtest header
#include <gtest/gtest.h>

// ROS header files
#include <ros/ros.h>
#include <ros/service_client.h>
#include "beginner_tutorials/ModifyText.h"

/**
 * @brief Tests whether service exists
 *
 * @param TalkerNodeTest            TESTSuite
 * @param testModifyTextServiceInit Test name
 */
TEST(TalkerNodeTest, testModifyTextServiceInit) {
  ros::NodeHandle nh;
  ros::ServiceClient client =
  nh.serviceClient<beginner_tutorials::ModifyText>("ModifyText");
  bool exists(client.waitForExistence(ros::Duration(5)));
  EXPECT_TRUE(exists);
}

/**
 * @brief Tests whether Modify text service is successful
 *
 * @param TalkerNodeTest            TESTSuite
 * @param testModifyTextServiceCall Test name
 */
TEST(TalkerNodeTest, testModifyTextServiceCall) {
  // Create a ros node handle
  ros::NodeHandle nh;

  // Create a client for ModifyText service
  ros::ServiceClient client =
  nh.serviceClient<beginner_tutorials::ModifyText>("ModifyText");

  // Check is service exists
  bool exists(client.waitForExistence(ros::Duration(5)));
  EXPECT_TRUE(exists);

  // Create a response and request for service
  beginner_tutorials::ModifyText::Request request;
  beginner_tutorials::ModifyText::Response response;

  request.inputString = "Update Text";

  // Call the service with updated text and check if status is set to true
  bool result = client.call(request, response);
  EXPECT_TRUE(result);
  EXPECT_TRUE(response.status);
}
