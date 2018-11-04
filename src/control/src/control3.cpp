//试gazebo是否能从cmd_pose话题接受位置信息，不能
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
//#include <mavros_msgs/CommandBool.h>
//#include <mavros_msgs/SetMode.h>
//#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>
#include <string>
#include <sstream>
#include <iostream>
//#include <geometry_msgs/Twist.h>
//#include <geometry_msgs/PoseStamped.h>
//#include<geometry_msgs/Vector3Stamped.h>

/*
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}
*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "posetest");
    ros::NodeHandle nh;

    //ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("cmd_pose", 1000);//定义叫做 local_pos_pub 的对象
    //ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    //ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(1);
    geometry_msgs::PoseStamped pose;//定义调用类 PoseStamped 成员的对象 
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    int cont = 0;
  while (ros::ok())
  {
   
    if( cont % 2 == 1) 
    {
        //pose.pose.position.x = 0;
        //pose.pose.position.y = 0;
        pose.pose.position.z = 8;
    };
    if( cont % 2 == 0)
    {
        //pose.pose.position.x = 0;
        //pose.pose.position.y = 0;
        pose.pose.position.z = 0;
    };

    local_pos_pub.publish(pose);//vel_control调用publisher的成员函数publish，发布消息
    ROS_INFO_STREAM("Sending cmd:"<<"z="<<pose.pose.position.z);
    cont++;
    rate.sleep();

  }
  return 0;
}
