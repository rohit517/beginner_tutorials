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
 *  @file    talker.cpp
 *  @author  Rohitkrishna Nambiar (rohith517)
 *  @date    10/28/2018
 *  @version 1.0
 *
 *  @brief ROS Beginner tutorials
 *
 *  @section DESCRIPTION
 *
 *  Source file to implement publisher node.
 *
 */

// C++ header
#include <sstream>

// ROS header
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/ModifyText.h"

// String message to publish
std::string publishMessage = "Welcome to ENPM808X ";

/**
 * @brief  Function to update text for ModifyText service
 * @param  request    Request data sent to service
 * @param  response   Response data by the service
 * @return bool
 */
bool updateText(beginner_tutorials::ModifyText::Request& request,
                beginner_tutorials::ModifyText::Response& response) {

  if (!request.inputString.empty()) {
    publishMessage = request.inputString;
    response.outputString = "String modified to: " + publishMessage;
    response.status = true;
    ROS_WARN_STREAM("Output string updated.");
  }
  else {
    response.status = false;
    response.outputString = publishMessage;
    ROS_ERROR_STREAM(
        "Output string to be updated cannot be empty. String will not be updated.");
  }

  return true;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
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

  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  ros::Duration(2).sleep();  // sleep for half a second for launch
  ROS_DEBUG_STREAM("Starting Talker node");

  // Default frequency value
  int frequency = 10;

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
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  // Service for ModifyText
  ros::ServiceServer service = n.advertiseService("ModifyText", updateText);

  // Check and set publish frequency
  if (argc == 2) {
    int freqArg = atoi(argv[1]);
    if (freqArg > 0) {
      // Set frequency to value of argument
      frequency = atoi(argv[1]);
      ROS_INFO("Publish frequency set to %d Hz.", frequency);
    }
    else if (freqArg == 0) {
      ROS_WARN("Publish frequency set to default value of %d Hz.",
                      frequency);
    }
    else {
      ROS_FATAL("Cannot start talker node with negative frequency of %d Hz.",
          frequency);
      return -1;
    }
  }
  else {
    ROS_INFO("NO arguments received. Publish frequency set to %d Hz.",
             frequency);
  }

  ros::Rate loop_rate(frequency);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << publishMessage << " " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
