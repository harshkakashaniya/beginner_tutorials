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
 *  @file    Talker.cpp
 *  @author  Harsh Kakashaniya
 *  @date    12/04/2018
 *  @version 1.2
 *
 *  @brief UMD ENPM 808X, ROS tutorials.
 *
 *  @Description DESCRIPTION
 *
 *  This file is used to generate message to a particular topic.
 *
 */
  // tf library to make a TransformBroadcaster
  #include <tf/transform_broadcaster.h>
  // C++ librarys
  #include <sstream>
  #include <iostream>
  // ROS libraries
  #include "ros/ros.h"
  #include "std_msgs/String.h"
  // service file
  #include "beginner_tutorials/change_string.h"

// default string of message in structure
struct str_msg {
  std::string message;
};
str_msg MessageObj;
/**
 *   @brief changing string with the help of service
 *
 *
 *   @param res
 *   @param req
 *   @return bool
 */
bool chg_str(beginner_tutorials::change_string::Request  &req,
beginner_tutorials::change_string::Response &res) {
    MessageObj.message = req.newString;
    res.responseString = "String Updated";
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
  ros::NodeHandle nh;
  MessageObj.message = "I am counting 10 numbers per second and reached ";
  int frequency = 10;  // default value of frequency if not set by argument
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
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

  //  taking in argument and converting it from string to int
  if (argc == 2) {
    frequency = atoi(argv[1]);
    }


  ros::Rate loop_rate(frequency);  // looping with 10 Hz frequency
  /**
   * Advertising service server so that when this node is functional service
   * can be called and it can do required changes in the code.
   */
  ros::ServiceServer change_string = nh.advertiseService("change_string"
                                                          , chg_str);
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;  // initiate count to be zero
  // checking if frequency is negative
  if (frequency <= 0) {
      ROS_FATAL_STREAM_ONCE("Frequency given is negative.Change"<<
                          "frequency to positive");
    }
    // Making object of TransformBroadcaster
    static tf::TransformBroadcaster br;
    tf::Transform transform;  //  object for transform function
    tf::Quaternion q;
  // checking status of ROS this will be false is any error or Ctrl+C
  while (ros::ok() && frequency > 0) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
     double X = 5*cos(ros::Time::now().toSec());  // X for circle with radius 5
     double Y = 5*sin(ros::Time::now().toSec());  // Y for circle with radius 5
     int time = ros::Time::now().toSec();  // time in int for modulus
     int Z = (time%10)-5;  // value of Z changes from -5 to 5
     transform.setOrigin(tf::Vector3(X, Y, Z));  // set origin
     q.setRPY(0, 0, 1.0);
     transform.setRotation(q);
     br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                          "world", "talk"));
     std_msgs::String msg;
     std::stringstream ss;
     ss << MessageObj.message << count;
     msg.data = ss.str();
     // To inform user about the frequency of debug message
     ROS_DEBUG_STREAM_THROTTLE(1, "Frequency set to 10 Hz");
     // Printing required message given by service or default
     ROS_INFO("%s", msg.data.c_str());
     // To warn user that if he forgot to close the node it is eating memory.

     if (count > 100) {
       ROS_WARN_STREAM_THROTTLE(2, "Number of message greater than 100");
      }
     // If message empty means error of not giving desired input
     if (MessageObj.message == "") {
       ROS_ERROR_STREAM_THROTTLE(5, "Empty Message,String Expected");
     }
    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
     chatter_pub.publish(msg);  // Publishing Message with publish

     ros::spinOnce();  //  to output message
     count++;  // increment of count
     loop_rate.sleep();  // to run loop till ROS is OK!!
  }
  return 0;
}
