#include <ros/ros.h>
#include "control/Point9.h"
#include <geometry_msgs/Point.h>


ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher pub3;


void transCallback(const control::Point9::ConstPtr& msg)
{
    geometry_msgs::Point goal1;
    geometry_msgs::Point goal2;
    geometry_msgs::Point goal3;

    goal1.x = msg->points[0].x;
    goal1.y = msg->points[0].y;
    goal1.z = msg->points[0].z;
    goal2.x = msg->points[1].x;
    goal2.y = msg->points[1].y;
    goal2.z = msg->points[1].z;
    goal3.x = msg->points[2].x;
    goal3.y = msg->points[2].y;
    goal3.z = msg->points[2].z;
    
    pub1.publish(goal1);
    pub2.publish(goal2);
    pub3.publish(goal3);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "transform");
    ros::NodeHandle nh;
    pub1 = nh.advertise<geometry_msgs::Point>("/uav1/pose_goal", 1000);
    pub2 = nh.advertise<geometry_msgs::Point>("/uav2/pose_goal", 1000);
    pub3 = nh.advertise<geometry_msgs::Point>("/uav3/pose_goal", 1000);
    ros::Subscriber sub = nh.subscribe("/trans", 1000, transCallback);//订阅Point9类型的数
    ros::spin();
    return 0;
}