//给点飞行标准程序
#include <ros/ros.h>
#include <string>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#define MAX_VEL 3

using namespace std;
using namespace ros;
using namespace geometry_msgs;

bool goal_flag = false;
ros::Publisher vel_control;//定义调用速度话题的对象vel_control（全局）
geometry_msgs::Point goal;//定义消息类型的对象

geometry_msgs::Twist v(geometry_msgs::PoseStamped now)//接受的参数类型定义为 消息类型的对象
{
    //now为实时的当前位置，而不只是接收goal时的位置，每次now都是新的
    double delta_x = goal.x - now.pose.position.x;
    double delta_y = goal.y - now.pose.position.y;
    double delta_z = goal.z - now.pose.position.z;
    double P = 1;
    geometry_msgs::Twist twist;
    twist.linear.x = (P * delta_x) < MAX_VEL ? (P * delta_x) : MAX_VEL;
    twist.linear.y = (P * delta_y) < MAX_VEL ? (P * delta_y) : MAX_VEL;
    twist.linear.z = (P * delta_z) < MAX_VEL ? (P * delta_z) : MAX_VEL;
    return twist;
} 

void nowCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    geometry_msgs::Twist vel;
    if (goal_flag)
    {
        vel = v(*msg);//v接受的参数类型是PoseStamped类的对象,故对指针msg解引用
    }
    else
    {
        vel.linear.x = 0;
        vel.linear.y = 0;
        vel.linear.z = 0;
    }
    vel_control.publish(vel);
    
    if ((goal.x - msg->pose.position.x) * (goal.x - msg->pose.position.x) +
        (goal.y - msg->pose.position.y) * (goal.y - msg->pose.position.y) +
        (goal.z - msg->pose.position.z) * (goal.z - msg->pose.position.z) < 0.0001)
    {
        goal_flag = false;
    }
}

//每发布一次目标位置，goalCallback只执行一次，nowcallback会一次次一直执行
void goalCallback(const geometry_msgs::Point::ConstPtr& msg)//msg是指针类型的引用
{
    //当收到目标点，更新标志位。
    goal_flag = true;
    //且，订阅的目标位置要传递给全局的goal，否则到了函数v，其goal不是本次目标
    goal = *msg;
}

int main(int argc, char **argv)
{
  
    ros::init(argc, argv, "posecontrol2");
    ros::NodeHandle n;
    vel_control = n.advertise<geometry_msgs::Twist>("/uav1/cmd_vel", 1000);//调用nodehandel的成员advertise
    //类型为geometry_msgs::Twist 对象vel
    ros::Subscriber pose_now = n.subscribe("/uav1/ground_truth_to_tf/pose", 1000, nowCallback);//订阅当前位置
    //类型为 geometry_msgs::PoseStamped 对象now
    ros::Subscriber pose_goal = n.subscribe("/uav1/pose_goal", 1000, goalCallback);//订阅目标位置
    //类型为 geometry_msgs::Point 对象goal
    ros::spin();
    return 0;
}



