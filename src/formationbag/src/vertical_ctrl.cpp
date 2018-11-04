#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include "formationbag/Point4.h"
#include <math.h>


ros::Publisher posepub2;
ros::Publisher posepub3;
float d;
bool flag = false;

geometry_msgs::Point uav2_relapose(geometry_msgs::PoseStamped uav1_now)
{
    geometry_msgs::Point uav2_goal;
    uav2_goal.x = uav1_now.pose.position.x;
    uav2_goal.y = uav1_now.pose.position.y;
    uav2_goal.z = uav1_now.pose.position.z + d;    
    return uav2_goal;
}

geometry_msgs::Point uav3_relapose(geometry_msgs::PoseStamped uav1_now)
{
    geometry_msgs::Point uav3_goal;
    uav3_goal.x = uav1_now.pose.position.x;
    uav3_goal.y = uav1_now.pose.position.y;
    uav3_goal.z = uav1_now.pose.position.z + 2*d; 
    return uav3_goal;  
}

void centerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{ 
    geometry_msgs::Point uav2_goal;
    geometry_msgs::Point uav3_goal;
    uav2_goal= uav2_relapose(*msg);
    uav3_goal= uav3_relapose(*msg); 

    formationbag::Point4 uav1_goal;    
    geometry_msgs::PoseStamped uav1_now;
    double delta_x = uav1_goal.position.x - uav1_now.pose.position.x;
    double delta_y = uav1_goal.position.y - uav1_now.pose.position.y;
    double delta_z = uav1_goal.position.z - uav1_now.pose.position.z;
    //uav1已经到了的前提下，订阅一次param发布一次位置
    if((flag==true) && pow(delta_x,2) + pow(delta_y,2) + pow(delta_z,2) < 0.01)
    {
        posepub2.publish(uav2_goal);
        posepub3.publish(uav3_goal);
        ROS_INFO_STREAM("vertical  d="<<d);
    }
    flag = false;
}

void paramCallback(const formationbag::Point4::ConstPtr& msg)
{
    flag = true;
    d = msg->d;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"vertical_ctrl");
    ros::NodeHandle nh;

    //订阅终端的param话题  <formationbag::Point4>
    ros::Subscriber paramsub = nh.subscribe("/PC/vertical_param", 1000, paramCallback); 

    //订阅uav1的位置反馈话题  <geometry_msgs::PoseStamped>
    ros::Subscriber centersub = nh.subscribe("/uav1/ground_truth_to_tf/pose", 1000, centerCallback);

    posepub2 = nh.advertise<geometry_msgs::Point>("/uav2/pose_goal",1000);
    posepub3 = nh.advertise<geometry_msgs::Point>("/uav3/pose_goal",1000); 
    ros::spin();
    return 0;

}