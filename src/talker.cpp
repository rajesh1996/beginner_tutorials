/**
 * @file talker.cpp
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
#include <sstream>
#include <string>
#include <tf/transform_broadcaster.h>
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
// %EndTag(MSG_HEADER)%
#include "beginner_tutorials/changeString.h"

extern std::string custom_msg = "I love robots";

/**
 * @fn bool newString(beginner_tutorials::changeString::Request&, beginner_tutorials::changeString::Response&)
 * @brief Service callback function which takes a string and returns it
 *
 * @param req
 * @param res
 * @return true if succesfully executed
 */

bool newString(beginner_tutorials::changeString::Request &req,
               beginner_tutorials::changeString::Response &res) {
  ROS_INFO_STREAM("String in service:" << req.inputString);
  custom_msg = req.inputString;
  ROS_WARN_STREAM("USER HAS CHANGED THE STRING!!!!!!!");
  res.OutputString = req.inputString;
  return true;
}

void tfBroadcast() {
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(1.0,2.0,3.0));
  tf::Quaternion q;
  q.setRPY(1,1,0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"world","talk"));
}

/**
 * @fn int main(int, char**)
 * @brief demonstrates simple sending of messages over the ROS system
 * @param argc
 * @param argv
 * @return
 */

int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
// %Tag(INIT)%
  ros::init(argc, argv, "talker");
// %EndTag(INIT)%

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
// %Tag(NODEHANDLE)%
  ros::NodeHandle n;
// %EndTag(NODEHANDLE)%

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
// %Tag(PUBLISHER)%
  ros::Publisher chatter_pub = n.advertise < std_msgs::String
      > ("chatter", 1000);

  // Advertise the service
  ros::ServiceServer service = n.advertiseService("changeString", newString);
// %EndTag(PUBLISHER)%
  int frequency_rate = 0;
  n.getParam("/freq_rate", frequency_rate);
  ROS_INFO_STREAM("Current frequency set: " << frequency_rate);
  if (frequency_rate > 0) {
    ROS_INFO_STREAM("Recieved user frequency");
  } else if (frequency_rate < 0) {
    ROS_FATAL_STREAM("Frequency cannot be less than zero");
    ROS_WARN_STREAM("Setting default frequency");
    frequency_rate = 8;
  } else if (frequency_rate == 0) {
    ROS_FATAL_STREAM("Frequency cannoot be zero");
    ROS_WARN_STREAM("Setting default frequency");
    frequency_rate = 8;
  }

// %Tag(LOOP_RATE)%
  ros::Rate loop_rate(frequency_rate);
// %EndTag(LOOP_RATE)%

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
// %Tag(ROS_OK)%
  int count = 0;
  while (ros::ok()) {
// %EndTag(ROS_OK)%
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
// %Tag(FILL_MESSAGE)%
    std_msgs::String msg;

    std::stringstream ss;
    ss << custom_msg << count;
    msg.data = ss.str();
// %EndTag(FILL_MESSAGE)%

// %Tag(ROSCONSOLE)%
    ROS_INFO("%s", msg.data.c_str());
    ROS_DEBUG_STREAM("Sending message:" << count);
// %EndTag(ROSCONSOLE)%

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
// %Tag(PUBLISH)%
    chatter_pub.publish(msg);
// %EndTag(PUBLISH)%
    tfBroadcast();
// %Tag(SPINONCE)%
    ros::spinOnce();
// %EndTag(SPINONCE)%

// %Tag(RATE_SLEEP)%
    loop_rate.sleep();
// %EndTag(RATE_SLEEP)%
    ++count;
  }
  ROS_FATAL_STREAM("Execution talker done!!!!!!");
  return 0;
}
// %EndTag(FULLTEXT)%
