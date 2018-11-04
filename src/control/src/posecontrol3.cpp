//师兄修改版给点飞行程序
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#define MAV_VEL 3
using namespace ros;
using namespace geometry_msgs;

bool goal_flag = false;
Publisher vel_control;
Twist twist;
Point goal;
int sign(double x)
{
    if (x > 0)
    {
        return 1;
    }
    else if(x< 0)
    {
        return -1;
    }
    else 
    {
        return 0;
    }
}
Twist v(PoseStamped now)
{
    double delta_x = goal.x - now.pose.position.x;
    double delta_y = goal.y - now.pose.position.y;
    double delta_z = goal.z - now.pose.position.z;
    double P = 1;
    if ( goal_flag && pow( delta_x , 2) + pow( delta_y , 2) + pow( delta_z , 2) > 0.01)
    {
        twist.linear.x = (abs(P*delta_x) < MAV_VEL)? P*delta_x: sign(P*delta_x)*MAV_VEL;//速度在-3~3前提下为P*△x
        twist.linear.y = (abs(P*delta_y) < MAV_VEL)? P*delta_y: sign(P*delta_y)*MAV_VEL;
        twist.linear.z = (abs(P*delta_z) < MAV_VEL)? P*delta_z: sign(P*delta_z)*MAV_VEL;
    }
    else
    {
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.linear.z = 0;
        goal_flag = false;
    }
    return twist;
}

void nowCallback(const PoseStamped::ConstPtr& msg)//定义引用:msg
{
    twist = v(*msg);       
    // double a,b,d;
    // a = (*msg).pose.position.x - goal.x;
    // b = (*msg).pose.position.y - goal.y;
    // d = sqrt(a*a+b*b);
    // if( d < 0.01 )
    // {
        vel_control.publish(twist);
    //}
    ROS_INFO("hello ros");
}

//每发布一次目标位置，goalCallback只执行一次，nowcallback会一次次执行
void goalCallback(const Point::ConstPtr& msg)
{
    goal_flag = true;
    goal = *msg;//将Point类型的消息传递给全局变量goal 
}

int main(int argc, char **argv)
{
    init (argc,argv,"posecontrol3");
    NodeHandle n;
    vel_control = n.advertise<Twist>("cmd_vel",1000);
    Subscriber pose_now = n.subscribe("ground_truth_to_tf/pose",1000,nowCallback);
    //<PoseStamped>
    Subscriber pose_goal = n.subscribe("pose_goal",1000,goalCallback);
    //<Point>
    spin();
    return 0;
}