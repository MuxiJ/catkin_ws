//此程序为全局，其话题须指定命名空间
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include "circlebag/Point5.h"
#include <math.h>

ros::Publisher posepub2;
ros::Publisher posepub3;
geometry_msgs::PoseStamped uav1_now;
geometry_msgs::PoseStamped uav2_now;
ros::Time time0;
float r = 1, rate = 0;

//输入5个参数，输出uav2位置
geometry_msgs::Point uav2_relapose(geometry_msgs::PoseStamped uav1_now)
{
    geometry_msgs::Point uav2_goal;
    float time1 = ros::Time::now().toSec() - time0.toSec();
//     double x,y,d;
//     x = (uav2_now).pose.position.x - uav2_goal.x;
//     y = (uav2_now).pose.position.y - uav2_goal.y;
//     d = sqrt(x*x+y*y);
//     if( d < 0.01 )
//    {
    if (uav1_now.pose.position.z > 0.1)
    {
        uav2_goal.x = r*sin( rate * time1) + uav1_now.pose.position.x;
        uav2_goal.y = r*cos( rate * time1) + uav1_now.pose.position.y;
        uav2_goal.z = uav1_now.pose.position.z;  
    }
    return uav2_goal;
}

//输入5个参数，输出uav3位置
geometry_msgs::Point uav3_relapose(geometry_msgs::PoseStamped uav1_now)
{
    geometry_msgs::Point uav3_goal;
    float time1 = ros::Time::now().toSec() - time0.toSec();
    if (uav1_now.pose.position.z > 0.1)
    {
        uav3_goal.x = r*sin( rate * time1+3.14159) + uav1_now.pose.position.x;
        uav3_goal.y = r*cos( rate * time1+3.14159) + uav1_now.pose.position.y;
        uav3_goal.z = uav1_now.pose.position.z;       
        
    }
    return uav3_goal;
}

//输入旧位置，输出新位置
geometry_msgs::Point NextPoint(geometry_msgs::Point msg)
{
    static int cnt = 0;

    cnt++;
}

//传入uav1位置msg，当uav2、3到了，发其新位置。(*msg是实时的当前uav1位置，且callback一直执行)
void centerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    
    geometry_msgs::Point uav2_goal;
    geometry_msgs::Point uav3_goal;
    uav1_now = (*msg);
    uav2_goal = uav2_relapose(*msg);//实时的新位置
    uav3_goal = uav3_relapose(*msg); 

    double x,y,d;
    x = (uav2_now).pose.position.x - uav2_goal.x;
    y = (uav2_now).pose.position.y - uav2_goal.y;
    d = sqrt(x*x+y*y);
    if( d < 0.01 )
    {
        uav2_goal = NextPoint(uav2_goal);

        posepub2.publish(uav2_goal);//发布uav2的位置(一直发),频率难道是spin频率？
        posepub3.publish(uav3_goal);
    }

}

//执行一次,传入消息里r,rate,传到全局的r,rate去
void sizeCallback(const circlebag::Point5::ConstPtr& msg)
{
    r = msg->r;
    rate = msg->rate;
}

void uav2Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    double a,b,r;
    a = (*msg).pose.position.x - uav1_now.pose.position.x;
    b = (*msg).pose.position.y - uav1_now.pose.position.y;
    r = sqrt(a*a+b*b);
    uav2_now = (*msg);
    ROS_INFO_STREAM("center="<<uav1_now.pose.position.x<<" "<<uav1_now.pose.position.y<<"  r="<<r);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "circle");
    ros::NodeHandle nh;
    ros::Time time0 = ros::Time::now();
    //订阅全局的size话题  <circlebag::Point5>
    ros::Subscriber sizesub = nh.subscribe("/size", 1000, sizeCallback); 
    //订阅uav1、2的位置反馈话题  <geometry_msgs::PoseStamped>
    ros::Subscriber centersub = nh.subscribe("/uav1/ground_truth_to_tf/pose", 1000, centerCallback);
    ros::Subscriber uav2sub = nh.subscribe("/uav2/ground_truth_to_tf/pose", 1000, uav2Callback);

    //定义发布Publisher的对象posepub的值,为uav2、3的位置  <geometry_msgs::Point>
    posepub2 = nh.advertise<geometry_msgs::Point>("/uav2/pose_goal",1000);
    posepub3 = nh.advertise<geometry_msgs::Point>("/uav3/pose_goal",1000);
    ros::spin();
    return 0;
}