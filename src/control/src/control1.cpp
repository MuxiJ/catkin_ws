//cmd_vel控仨
#include "std_msgs/String.h"
#include <sstream>
#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>
#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
//#include<geometry_msgs/Vector3Stamped.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "v");
  ros::NodeHandle n;
  ros::Publisher vel_Control1 = n.advertise<geometry_msgs::Twist>("/uav1/cmd_vel", 1000);//定义叫做vel_control的对象
  ros::Publisher vel_Control2 = n.advertise<geometry_msgs::Twist>("/uav2/cmd_vel", 1000);
  ros::Publisher vel_Control3 = n.advertise<geometry_msgs::Twist>("/uav3/cmd_vel", 1000);

  ros::Rate loop_rate(0.2);

  geometry_msgs::Twist msg;//定义调用类Twist成员的对象msg
  msg.linear.x = 0;
  msg.linear.y = 0;
  msg.linear.z = 0;

  int cont = 0;
  while (ros::ok())
  {
   
    if (cont % 2 == 1) msg.linear.z = 1;
    if (cont % 2 == 0) msg.linear.z = -1;
    
    vel_Control1.publish(msg);//vel_control调用publisher的成员函数publish，发布消息
    vel_Control2.publish(msg);
    vel_Control3.publish(msg);


    ROS_INFO_STREAM("Sending velocity cmd:"<<"linear.z="<<msg.linear.z);
    cont++;
    loop_rate.sleep();
    
    //msg.vector.z = 1;
    //vel_Control.publish(msg);
    //loop_rate.sleep();

    //msg.vector.z = -1;
    //vel_Control.publish(msg);
    //loop_rate.sleep();
  }
  return 0;
}
