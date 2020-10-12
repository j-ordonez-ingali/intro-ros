#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */


void int32Callback(const std_msgs::Int32::ConstPtr& msg)
{
  ROS_INFO("Received message: [%d]",msg->data);

}



int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "simple_int32_subscriber");// node name 


  ros::NodeHandle node;


  ros::Subscriber sub = node.subscribe("numbers_int32",10, int32Callback);

  ros::spin();

  //numbers_int32-->topic name that this node will be subscribed, 10 -->queue size



  ROS_INFO("this printing is not sent");

  


  return 0;
}

