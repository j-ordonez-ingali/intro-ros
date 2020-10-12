#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <math.h>
#include <sstream>

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "cos_publisher"); /////// argc --> int intput arguments, argv --> char input arguments , cos_publisher --> node name

 
  ros::NodeHandle node;  ////////// node is the name of node handler,this can configure a node


  ros::Publisher float64_pub = node.advertise<std_msgs::Float64>("numbers_float", 1000); // node.advertise -->initializing publisher

//std_msgs::Int32 -->message type, numbers_int32 -->topic name,  10-->queue size   

  ros::Rate loop_rate(30);///////  1Hz 1 message per second

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  float count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::Float64 message; //////creating a message


    message.data = cos(count);  ////assign message data

    ROS_INFO("Published message: %u", message.data);  ////ROS_INFO is similar to print function,

 
    float64_pub.publish(message);///////////given the publisher int32_pub, publish(message) send message "message"

    ros::spinOnce();   //keep alive

    loop_rate.sleep(); //sleep
    count=count+((1/45)*3.141592);
  }
  return 0;
}
