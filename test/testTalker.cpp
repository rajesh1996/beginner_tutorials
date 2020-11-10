/**
 * @file testTalker.cpp
 * @author Rajeshwar N S
 * @copyright 2020 Rajeshwar N S
 * @brief Source code for publisher - Sending a sample message
 */

/**
 *MIT License
 *Copyright (c) 2020 Rajeshwar N S
 *Permission is hereby granted, free of charge, to any person obtaining a copy
 *of this software and associated documentation files (the "Software"), to deal
 *in the Software without restriction, including without limitation the rights
 *to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *copies of the Software, and to permit persons to whom the Software is
 *furnished to do so, subject to the following conditions:
 *The above copyright notice and this permission notice shall be included in all
 *copies or substantial portions of the Software.
 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *SOFTWARE.
 */

#include <ros/service_client.h>
#include <gtest/gtest.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/changeString.h"

/**
 * @def TEST(Talker_test, test_talk)
 * @brief Check for the service
 */

TEST(Talker_test, test_talk) {
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient < beginner_tutorials::changeString
      > ("changeString");
  // check for service
  bool exists(client.waitForExistence(ros::Duration(5)));
  EXPECT_TRUE(exists);
}
/**
 * @class TEST(Talker_testt, test_talkk)
 * @brief tests for change of string service
 *
 */
TEST(Talker_testt, test_talkk) {
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient < beginner_tutorials::changeString
      > ("changeString");
  beginner_tutorials::changeString srv;
  srv.request.inputString = "Mic Testing";
  client.call(srv.request, srv.response);
  // check for the new changed sttring
  EXPECT_EQ("Mic Testing", srv.response.OutputString);
}
/**
 * @fn int main(int, char**)
 * @brief Main function that run all test cases
 * 
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "testTalker");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
