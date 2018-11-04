//自己写的给点飞行鬼程序
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <string>
#include <sstream>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point.h>
#include "std_msgs/String.h"
bool flag = 0;

ros::Publisher vel_control;//定义调用速度话题的对象vel_control（全局）
geometry_msgs::Twist msg;
geometry_msgs::PoseStamped now;//定义 调用 类poseStamped的成员 的对象now 
geometry_msgs::Point goal;//定义对象

// void vx(double)//输入当前位置，处理成速度
// {
//     if (goal.x - now.pose.position.x > 0 ) msg.linear.x= 0.5;
// }
// void vy(double)
// {
//     if (goal.y - now.pose.position.y > 0 ) msg.linear.y= 0.5;
// }
// void vz(double)
// {
//     if (goal.z - now.pose.position.z > 0 ) msg.linear.z= 0.5;
// }


void v(double , double, double)
{
    if (goal.x - now.pose.position.x > 0 ) msg.linear.x= 0.5;
    else if (goal.y - now.pose.position.y > 0 ) msg.linear.y= 0.5;
    else if (goal.z - now.pose.position.z > 0 ) msg.linear.z= 0.5;
    else flag = false;
}


void nowCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    //while( ros(ok) && (flag) )
    while(flag)
    {
        if(0 == flag) break;   
        // vx(goal.x);
        // vy(goal.y);
        // vz(goal.z);
        v(goal.x, goal.y, goal.z);
        vel_control.publish(msg);//调用发布速度的函数，发布速度话题
        ROS_INFO_STREAM("Send pose:"<<goal.x<<","<<goal.y<<","<<goal.z);
        ros::Rate loop_rate(10);
        loop_rate.sleep();
        
    }
}

void goalCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    flag = 1;
}

int main(int argc, char **argv)
{
  
    ros::init(argc, argv, "posecontrol");
    
    ros::NodeHandle n;
    vel_control= n.advertise<geometry_msgs::Twist>("/uav1/cmd_vel", 1000);//调用nodehandel的成员advertise
    ros::Subscriber pose_now = n.subscribe("/uav1/ground_truth_to_tf/pose", 1000, nowCallback);//订阅当前位置
    //类型为geometry_msgs::PoseStamped
    ros::Subscriber pose_goal = n.subscribe("/uav1/pose_goal", 1000, goalCallback);//订阅目标位置
    //类型为geometry_msgs::Point
    ros::spin();
    return 0;
}



