//5个数
#include <ros/ros.h>
#include "circlebag/Point5.h"
#include <geometry_msgs/Point.h>

ros::Publisher posepub;
ros::Publisher sizepub;

void paramCallback(const circlebag::Point5Ptr& msg)
{
    geometry_msgs::Point pose;
    circlebag::Point5 size;

    pose.x = msg->position.x;   //msg是指针类型的引用。指针->成员
    pose.y = (*msg).position.y; //先 *msg 解引用（取内存单元值）。再：对象.成员
    pose.z = (*msg).position.z;
    size.r = msg->r;
    size.rate =(*msg).rate;  
    
    posepub.publish(pose);
    sizepub.publish(size);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "transform1");
    ros::NodeHandle nh;
    ros::Subscriber paramsub = nh.subscribe("/circle_param", 1000, paramCallback);
    posepub = nh.advertise<geometry_msgs::Point>("/uav1/pose_goal", 1000);
    sizepub = nh.advertise<circlebag::Point5>("/size", 1000);
    ros::spin();
    return 0;
}
