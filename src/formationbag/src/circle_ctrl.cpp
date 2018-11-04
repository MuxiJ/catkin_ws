//此程序为全局，其话题须指定命名空间
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include "formationbag/Point5.h"
#include <math.h>

ros::Publisher posepub2;
ros::Publisher posepub3;
geometry_msgs::Point uav1_goal;
ros::Time time0;
float r, rate;
bool flag = false;

//输入5个参数，输出uav2位置，但当订阅到param才发布
geometry_msgs::Point uav2_relapose(geometry_msgs::PoseStamped uav1_now)
{
    geometry_msgs::Point uav2_goal;
    float time1 = ros::Time::now().toSec() - time0.toSec();
    // if (uav1_now.pose.position.z > 0.1)
    // {
        uav2_goal.x = r * sin(rate * time1) + uav1_now.pose.position.x;
        uav2_goal.y = r * cos(rate * time1) + uav1_now.pose.position.y;
        uav2_goal.z = uav1_now.pose.position.z;  
    //}
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

//传入uav1位置msg，发布uav2、3的位置。*msg是实时的当前uav1位置，且callback一直执行
void centerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

    geometry_msgs::Point uav2_goal;
    geometry_msgs::Point uav3_goal;
    uav2_goal= uav2_relapose(*msg);
    uav3_goal= uav3_relapose(*msg); 

    double delta_x = uav1_goal.x - msg->pose.position.x;
    double delta_y = uav1_goal.y - msg->pose.position.y;
    double delta_z = uav1_goal.z - msg->pose.position.z;
    //当uav1到了后，订阅到circle_param时才发布位置,否则飞机为静止   
    if((flag == true) && pow(delta_x, 2) + pow(delta_y, 2) + pow(delta_z, 2) < 0.07)
    {
        posepub2.publish(uav2_goal);//发布uav2的位置(一直发),频率难道是spin频率？
        posepub3.publish(uav3_goal);
        ROS_INFO_STREAM("circle  r="<<r<<"  rate="<<rate);
    }
    flag = false;
}

void uav1GoalCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    uav1_goal = *msg;
}

//订阅到一次就执行一次,传入消息里r,rate,传到全局的r,rate去
void paramCallback(const formationbag::Point5::ConstPtr& msg)
{
    flag = true;//终端必须得一直发circle使得flag一直1（-r 10）
    r = msg->r;
    rate = msg->rate;
}

void uav2Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    double a,b,r;
    a = (*msg).pose.position.x - uav1_goal.x;
    b = (*msg).pose.position.y - uav1_goal.y;
    r = sqrt(a*a+b*b);
    ROS_INFO_STREAM("center="<<uav1_goal.x<<" "<<uav1_goal.y<<"  r="<<r);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "circle_ctrl");
    ros::NodeHandle nh;
    ros::Time time0 = ros::Time::now();

    //订阅终端的param话题  <formationbag::Point5>
    ros::Subscriber paramsub = nh.subscribe("/PC/circle_param", 1000, paramCallback); 
    //订阅uav1的目标位置
    ros::Subscriber uav1_goal = nh.subscribe("/uav1/pose_goal", 1000, uav1GoalCallback);    
    //订阅uav1的位置反馈话题  <geometry_msgs::PoseStamped>
    ros::Subscriber centersub = nh.subscribe("/uav1/ground_truth_to_tf/pose", 1000, centerCallback);
    ros::Subscriber uav2sub = nh.subscribe("/uav2/ground_truth_to_tf/pose", 1000, uav2Callback);

    //定义发布Publisher的对象posepub的值,为uav2、3的位置  <geometry_msgs::Point>
    posepub2 = nh.advertise<geometry_msgs::Point>("/uav2/pose_goal",1000);
    posepub3 = nh.advertise<geometry_msgs::Point>("/uav3/pose_goal",1000);
    
    ros::spin();
    return 0;
}