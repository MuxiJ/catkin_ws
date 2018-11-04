#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "param_demo");
    ros::NodeHandle n;
    ros::NodeHandle pn("~my_namespace");

    std::string s;
    int num;

//  被初始化变量类型   参数名     被初始化变量名     参数值
    n.param<std::string>("param1", s, "default_string");
    pn.param<int>("param2", num, 666);

//    n.setParam("param1","default_string"); 
//    pn.setParam("param2",2333);   

    ROS_INFO("\nparam1:  %s\n", s.c_str());//输出初始化值
    ROS_INFO("param2:  %d\n\n", num);//输出初始化值

    ros::Rate loop_rate(0.5);

    while (ros::ok())
    {
        n.getParam("param1",s);   
        ROS_INFO("param1:  %s\n", s.c_str());
        pn.getParam("param2",num);
        ROS_INFO("param2:  %d\n\n", num);

        ros::spinOnce();
        loop_rate.sleep();
    }
}