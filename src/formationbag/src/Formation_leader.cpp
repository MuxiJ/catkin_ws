#include <ros/ros.h>
#include "formationbag/Point5.h"
#include "formationbag/Point4.h"
#include <geometry_msgs/Point.h>

ros::Publisher uav1pub;

void horizonCallback(const formationbag::Point4Ptr& msg)
{
    geometry_msgs::Point pose;

    pose.x = msg->position.x;  
    pose.y = (*msg).position.y; 
    pose.z = (*msg).position.z;
    
    uav1pub.publish(pose);
}

void verticalCallback(const formationbag::Point4Ptr& msg)
{
    geometry_msgs::Point pose;

    pose.x = msg->position.x;   
    pose.y = (*msg).position.y;
    pose.z = (*msg).position.z;
    
    uav1pub.publish(pose);
}

void circleCallback(const formationbag::Point5Ptr& msg)
{
    geometry_msgs::Point pose;

    pose.x = msg->position.x;   //msg是指针类型的引用。指针->成员
    pose.y = (*msg).position.y; //先 *msg 解引用（取内存单元值）。再：对象.成员
    pose.z = (*msg).position.z;
    
    uav1pub.publish(pose);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Formation_leader");
    ros::NodeHandle nh;
    
    //<formationbag::Point4>    
    ros::Subscriber horizonsub = nh.subscribe("/PC/horizon_param", 1000, horizonCallback);
    //<formationbag::Point4>
    ros::Subscriber verticalsub = nh.subscribe("/PC/vertical_param", 1000, verticalCallback);    
    //<formationbag::Point5>
    ros::Subscriber circlesub = nh.subscribe("/PC/circle_param", 1000, circleCallback);

    uav1pub = nh.advertise<geometry_msgs::Point>("/uav1/pose_goal", 1000);
    ros::spin();
    return 0;
}
