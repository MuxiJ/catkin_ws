//PX4例程的移植，失败
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
//#include <mavros_msgs/CommandBool.h>
//#include <mavros_msgs/SetMode.h>
//#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>
#include <string>
#include <sstream>
#include <iostream>
//#include <geometry_msgs/Twist.h>
//#include <geometry_msgs/PoseStamped.h>
//#include<geometry_msgs/Vector3Stamped.h>

/*
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}
*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "holdheight");
    ros::NodeHandle nh;

    //ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/ground_truth_to_tf/pose", 1000);//定义叫做 local_pos_pub 的对象
    //ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    //ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
 
    // wait for FCU connection
  /*while(ros::ok() && current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
  */
    geometry_msgs::PoseStamped pose;//定义调用类 PoseStamped 成员的对象 
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 5;
    /*
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    */
    int state = 3;  
    ros::Time last_request = ros::Time::now();

    //延迟5s
    while (ros::ok())
    {
      /*  if( !current_state.armed )
        {
            if( arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle armed");
            }
        }
        if( current_state.mode != "OFFBOARD")
        {
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.success)
            {
                ROS_INFO("Offboard enabled");
            }
        } 
      */
        //if( (ros::Time::now() - last_request > ros::Duration(5.0))) break;
       
        ROS_INFO("aaa");
        ros::spinOnce();
        rate.sleep();
    }

    while(state--) //先取值后自减，执行3次
    {
        last_request = ros::Time::now();
        while(ros::ok()) 
        {
            if( (ros::Time::now() - last_request > ros::Duration(5.0))) break;//5s后跳出while
           
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            local_pos_pub.publish(pose);//调用发布函数
            
            ROS_INFO("position 1");
            ros::spinOnce();//一直调用发布函数
            rate.sleep(); //每0.05执行swhile一次，相当于1s发布20次     
        }
        
        last_request = ros::Time::now();
        while(ros::ok()) 
        {
            if( (ros::Time::now() - last_request > ros::Duration(5.0))) break;
           
            pose.pose.position.x = 5;
            pose.pose.position.y = 5;
            local_pos_pub.publish(pose);
           
            ROS_INFO("position 2");
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO_STREAM("state="<<state);

    }

  /*offb_set_mode.request.custom_mode = "AUTO.LAND";
    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.success)
    {
        ROS_INFO("AUTO.LAND enabled");
        last_request = ros::Time::now();
    }
  */

    return 0;
}

