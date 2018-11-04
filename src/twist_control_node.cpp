/**
 * @file twist_control_node.cpp
 * @brief offboard twist control node, include:takeoff,langing,position hold,twist_control
 * @maintainer liuqisheng_ws@163.com
 */

#include <ros/ros.h>
#include "std_srvs/Empty.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <iomanip> 
#include <cmath>

#include <opencv2/opencv.hpp>

mavros_msgs::State current_state;//飞控状态
geometry_msgs::PoseStamped uavCurrentLocalPose;//当前位姿
mavros_msgs::PositionTarget uav_vel_tar,uav_poshold_tar;//local_raw控制消息和定点控制消息
double uavRollENU, uavPitchENU, uavYawENU;//飞控ENU姿态

enum UAV_STATE { 
    UAV_NOTREADY, //非offboard模式
    UAV_TAKEOFF_START,//起飞状态 
    UAV_POS_HOLD,//定点状态
    UAV_2DVEL_CONTROL, //平面控速模式(定高)
    UAV_3DVEL_CONTROL,//3D控速模式
    UAV_LANDING_START, //开始降落状态
    UAV_LANDING//降落中
}uav_state;//用于状态既切换控制状态

double landoff_height=1.5;//期望的离地高度
double takeoff_height=landoff_height;//命令下发的飞行高度
double pos_hold_height=landoff_height;//定高的高度
ros::Time takeoff_start_time;//开始起飞的时间
int cmd_received_count=0;//连续收到指令数量
ros::Time last_cmd_time;//上一次接受到命令的时间
ros::Time cmd_start_time;//开始收到指令的时间
bool cmd_lost_flag=true;//连续控制指令中断标志

//以下是可选的控制标志,注意各标志位的优先关系,位置>速度>加速度
/*mavros_msgs::PositionTarget::IGNORE_PX|mavros_msgs::PositionTarget::IGNORE_PY|mavros_msgs::PositionTarget::IGNORE_PZ|
                                mavros_msgs::PositionTarget::IGNORE_VX|mavros_msgs::PositionTarget::IGNORE_VY|mavros_msgs::PositionTarget::IGNORE_VZ|
                                mavros_msgs::PositionTarget::IGNORE_AFX|mavros_msgs::PositionTarget::IGNORE_AFY|mavros_msgs::PositionTarget::IGNORE_AFZ|
                                mavros_msgs::PositionTarget::IGNORE_YAW|mavros_msgs::PositionTarget::IGNORE_YAW_RATE;*/
//定高模式(注意VZ不能忽略!!!)
#define VEL_2D_MODE  mavros_msgs::PositionTarget::IGNORE_PX|mavros_msgs::PositionTarget::IGNORE_PY|\
                     mavros_msgs::PositionTarget::IGNORE_AFX|mavros_msgs::PositionTarget::IGNORE_AFY|mavros_msgs::PositionTarget::IGNORE_AFZ|\
                     mavros_msgs::PositionTarget::IGNORE_YAW
//定点模式
#define POS_HOLD_MODE mavros_msgs::PositionTarget::IGNORE_VX|mavros_msgs::PositionTarget::IGNORE_VY|mavros_msgs::PositionTarget::IGNORE_VZ|\
                      mavros_msgs::PositionTarget::IGNORE_AFX|mavros_msgs::PositionTarget::IGNORE_AFY|mavros_msgs::PositionTarget::IGNORE_AFZ|\
                      mavros_msgs::PositionTarget::IGNORE_YAW_RATE
//空间速度模式
#define VEL_3D_MODE  mavros_msgs::PositionTarget::IGNORE_PX|mavros_msgs::PositionTarget::IGNORE_PY|mavros_msgs::PositionTarget::IGNORE_PZ|\
                     mavros_msgs::PositionTarget::IGNORE_AFX|mavros_msgs::PositionTarget::IGNORE_AFY|mavros_msgs::PositionTarget::IGNORE_AFZ|\
                     mavros_msgs::PositionTarget::IGNORE_YAW

//#define DEBUG_MODE//仿真调试模式开关


void state_cb(const mavros_msgs::State::ConstPtr& msg){//飞控状态消息回调
    current_state = *msg;
}

void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)//当前位姿消息回调
{
    uavCurrentLocalPose.pose = msg->pose;//更新当前位姿
    //Using ROS tf to get RPY angle from Quaternion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(uavCurrentLocalPose.pose.orientation, quat);//tf转换得到姿态
    tf::Matrix3x3(quat).getRPY(uavRollENU, uavPitchENU, uavYawENU);
}

void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg)//速度控制回调
{  
    if(cmd_received_count==0)//记录第一次收到控制命令时间
        cmd_start_time=ros::Time::now();
    cmd_received_count++;
    if((cmd_lost_flag)&&(ros::Time::now() - cmd_start_time > ros::Duration(2)))//连续2秒接到速度控制消息才认为控制指令状态正常
        cmd_lost_flag=false;
    last_cmd_time=ros::Time::now();
    if(((uav_state==UAV_3DVEL_CONTROL)||(uav_state==UAV_2DVEL_CONTROL)||(uav_state==UAV_POS_HOLD))&&(cmd_lost_flag==false)){//指令状态正常则切入速度控制模式,起飞降落状态下忽略速度控制命令
        if((msg->linear.z!=0)&&(uav_state!=UAV_3DVEL_CONTROL)){//3D控速
            uav_state=UAV_3DVEL_CONTROL;
            uav_vel_tar.type_mask=VEL_3D_MODE;
            std::cout<<"enter in UAV_3DVEL_CONTROL mode!!!"<<std::endl;
        }
        else if(uav_state!=UAV_2DVEL_CONTROL){//平面控速
            pos_hold_height=uavCurrentLocalPose.pose.position.z;
            uav_state=UAV_2DVEL_CONTROL;
            uav_vel_tar.type_mask=VEL_2D_MODE;
            std::cout<<"enter in UAV_2DVEL_CONTROL mode!!!pos_hold_height="<<pos_hold_height<<std::endl;
        }
    }

    uav_vel_tar.header.stamp=ros::Time::now();
    uav_vel_tar.header.frame_id="uav";
    if(msg->linear.z!=0){//3D控速
        uav_vel_tar.velocity.z=msg->linear.z;
    }
    else{//2D控速
        uav_vel_tar.position.z=pos_hold_height;
        uav_vel_tar.velocity.z=0;
    }
    uav_vel_tar.velocity.x=-msg->linear.y;//注意坐标转换
    uav_vel_tar.velocity.y=msg->linear.x;
    uav_vel_tar.yaw_rate=msg->angular.z;
}

bool auto_takeoff_service(std_srvs::Empty::Request &req,//自动起飞服务
                               std_srvs::Empty::Response &res)
{
  if( current_state.mode != "OFFBOARD" ){
      std::cout<<"not in offboard!!!"<<std::endl;
      return false;
  }
  if(current_state.armed==false){
      std::cout<<"not armed!!!"<<std::endl;
      return false;
  }
  takeoff_height=landoff_height+uavCurrentLocalPose.pose.position.z;//指令高度=地面高度+期望离地高度
  if(takeoff_height<landoff_height)
    takeoff_height=landoff_height;
  uav_poshold_tar.header.stamp=ros::Time::now();
  uav_poshold_tar.position.x=uavCurrentLocalPose.pose.position.x;
  uav_poshold_tar.position.y=uavCurrentLocalPose.pose.position.y;
  uav_poshold_tar.position.z=takeoff_height;
  uav_poshold_tar.yaw=uavYawENU;
  takeoff_start_time=ros::Time::now();
  uav_state=UAV_TAKEOFF_START;
  std::cout<<"auto takeoff start"<<std::endl;
  return true;
}

bool auto_landing_service(std_srvs::Empty::Request &req,//自动降落服务
                               std_srvs::Empty::Response &res)
{
//   if( current_state.mode != "OFFBOARD" ){
//       std::cout<<"not in offboard!!!"<<std::endl;
//       return false;
//   }
  if(current_state.armed==false){
      std::cout<<"not armed!!!"<<std::endl;
      return false;
  }
  uav_state=UAV_LANDING_START;
  std::cout<<"auto landing start"<<std::endl;
  return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_twist_control_node");
    ros::NodeHandle nh;

    //local pose订阅
    ros::Subscriber local_pos_sub=nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, local_pos_cb);
    //飞机状态订阅
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    //速度控制消息订阅
    ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 10, cmd_vel_cb);
    //local pose全局坐标下的速度控制,目前统一由setpoint_raw来替代完成
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);
    //local pose全局坐标下的位置控制,目前统一由setpoint_raw来替代完成
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    //setpoint_raw话题,根据消息设置不同,可提供全局坐标系下的控速控点,及机体坐标系下的控速
    ros::Publisher local_raw_tar_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local", 10);
    //px4解锁上锁服务,慎用,解锁上锁的权限最好交给遥控器
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    //px4模式切换服务,目前用于自动降落
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    //对外提供自动起飞服务,起飞高度由landoff_height指定
    ros::ServiceServer takeoff_service = nh.advertiseService("/auto_takeoff", auto_takeoff_service);
    //对外提供自动降落服务
    ros::ServiceServer landing_service = nh.advertiseService("/auto_landing", auto_landing_service);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    uav_vel_tar.header.stamp=ros::Time::now();
    uav_vel_tar.header.frame_id="uav";
    uav_vel_tar.coordinate_frame=mavros_msgs::PositionTarget::FRAME_BODY_NED;//设置为机体坐标系
    uav_vel_tar.type_mask=VEL_2D_MODE;
    uav_vel_tar.position.x=0;
    uav_vel_tar.position.y=0;
    uav_vel_tar.position.z=takeoff_height;

    uav_vel_tar.velocity.x=0;
    uav_vel_tar.velocity.y=0;
    uav_vel_tar.velocity.z=0;

    uav_vel_tar.acceleration_or_force.x=0;
    uav_vel_tar.acceleration_or_force.y=0;
    uav_vel_tar.acceleration_or_force.z=0;

    uav_vel_tar.yaw=0;
    uav_vel_tar.yaw_rate=0;

    uav_poshold_tar.header.stamp=ros::Time::now();
    uav_poshold_tar.header.frame_id="uav";
    uav_poshold_tar.coordinate_frame=mavros_msgs::PositionTarget::FRAME_LOCAL_NED;//设置为全局坐标系
    uav_poshold_tar.type_mask=POS_HOLD_MODE;
    uav_poshold_tar.position.x=0;
    uav_poshold_tar.position.y=0;
    uav_poshold_tar.position.z=takeoff_height;


    uav_state=UAV_NOTREADY;//初始化控制状态
    mavros_msgs::SetMode offb_set_mode;//设置要切换的模式,此处用于自动降落
    offb_set_mode.request.custom_mode = "AUTO.LAND";

    ros::Time last_request = ros::Time::now();

    #ifdef DEBUG_MODE//如果是在电脑上进行调试,则打开按键控制窗口方便控制起飞和降落
    cv::namedWindow("Key");
    #endif
    ROS_INFO("px4_twist_control_node is up!");


    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" ||!current_state.armed){//不在OFFBOARD模式或者未解锁则切到准备状态
            uav_state=UAV_NOTREADY;
        }
        if(((uav_state==UAV_3DVEL_CONTROL)||(uav_state==UAV_2DVEL_CONTROL))
            &&(ros::Time::now() - last_cmd_time > ros::Duration(0.5))){//0.5s内未接收到速度控制指令则原地定点
            cmd_received_count=0;
            cmd_lost_flag=true;
            uav_state=UAV_POS_HOLD;
            std::cout<<"enter in UAV_POS_HOLD mode!!!pos_hold_height="<<pos_hold_height<<std::endl;
            pos_hold_height=uavCurrentLocalPose.pose.position.z;
            uav_poshold_tar.header.stamp=ros::Time::now();
            uav_poshold_tar.position.x=uavCurrentLocalPose.pose.position.x;
            uav_poshold_tar.position.y=uavCurrentLocalPose.pose.position.y;
            uav_poshold_tar.position.z=pos_hold_height;
            uav_poshold_tar.yaw=uavYawENU;
        }
        switch(uav_state){
            case UAV_NOTREADY://准备状态
                        takeoff_height=landoff_height+uavCurrentLocalPose.pose.position.z;
                        if(takeoff_height<landoff_height)
                            takeoff_height=landoff_height;
                        uav_poshold_tar.header.stamp=ros::Time::now();
                        uav_poshold_tar.position.x=uavCurrentLocalPose.pose.position.x;
                        uav_poshold_tar.position.y=uavCurrentLocalPose.pose.position.y;
                        uav_poshold_tar.position.z=takeoff_height;
                        uav_poshold_tar.yaw=uavYawENU;
                        local_raw_tar_pub.publish(uav_poshold_tar);

                        if( current_state.mode == "OFFBOARD" && current_state.armed){//切offboard转入起飞状态
                            takeoff_start_time=ros::Time::now();
                            uav_state=UAV_TAKEOFF_START;
                            std::cout<<"auto takeoff start,takeoff height="<<takeoff_height<<std::endl;
                        }
                        break;
            case UAV_TAKEOFF_START://起飞状态
                        uav_poshold_tar.header.stamp=ros::Time::now();
                        local_raw_tar_pub.publish(uav_poshold_tar);
                        if((uavCurrentLocalPose.pose.position.z>takeoff_height-0.1)&&//到达起飞高度且保证至少5s的起飞时间
                            (ros::Time::now() - takeoff_start_time > ros::Duration(5.0))){//转入定点模式
                                uav_state=UAV_POS_HOLD;
                                std::cout<<"enter in UAV_POS_HOLD mode!!!"<<std::endl;
                        }
                        break;
            case UAV_POS_HOLD://定点模式
                        //uav_poshold_tar.position.z=takeoff_height;
                        uav_poshold_tar.header.stamp=ros::Time::now();
                        local_raw_tar_pub.publish(uav_poshold_tar);
                        break;
            case UAV_2DVEL_CONTROL://平面控速模式
                        local_raw_tar_pub.publish(uav_vel_tar);
                        break;
            case UAV_3DVEL_CONTROL://3D控速模式
                        local_raw_tar_pub.publish(uav_vel_tar);
                        break;
            case UAV_LANDING_START://开始降落模式
                        if( current_state.mode != "AUTO.LAND"){
                            offb_set_mode.request.custom_mode = "AUTO.LAND";
                            if( set_mode_client.call(offb_set_mode) &&
                                offb_set_mode.response.mode_sent){
                                ROS_INFO("AUTO.LAND enabled");
                            }
                            last_request = ros::Time::now();
                            uav_state=UAV_LANDING;
                        }
                        break;
            case UAV_LANDING://正在降落模式(实际没存在必要)
                        if( current_state.mode != "AUTO.LAND"&&
                            (ros::Time::now() - last_request > ros::Duration(5.0))){
                            offb_set_mode.request.custom_mode = "AUTO.LAND";
                            if( set_mode_client.call(offb_set_mode) &&
                                offb_set_mode.response.mode_sent){
                                ROS_INFO("AUTO.LAND enabled");
                            }
                            last_request = ros::Time::now();
                        }
                        break;
            default:break;
        }
        #ifdef DEBUG_MODE//如果是在电脑上进行仿真调试,则打开按键控制窗口方便控制起飞和降落
        char key=cv::waitKey(5);
        switch(key){
            case 'q'://起飞命令
                if( current_state.mode != "OFFBOARD" ){
                    std::cout<<"not in offboard!!!"<<std::endl;
                    break;
                }
                if(current_state.armed==false){
                    std::cout<<"not armed!!!"<<std::endl;
                    break;
                }
                takeoff_height=landoff_height+uavCurrentLocalPose.pose.position.z;
                if(takeoff_height<landoff_height)
                    takeoff_height=landoff_height;
                uav_poshold_tar.header.stamp=ros::Time::now();
                uav_poshold_tar.position.x=uavCurrentLocalPose.pose.position.x;
                uav_poshold_tar.position.y=uavCurrentLocalPose.pose.position.y;
                uav_poshold_tar.position.z=takeoff_height;
                uav_poshold_tar.yaw=uavYawENU;
                local_raw_tar_pub.publish(uav_poshold_tar);
                takeoff_start_time=ros::Time::now();
                uav_state=UAV_TAKEOFF_START;
                std::cout<<"auto takeoff start"<<std::endl;
                break;
            case 's'://降落命令
                // if( current_state.mode != "OFFBOARD" ){
                //     std::cout<<"not in offboard!!!"<<std::endl;
                //     break;
                // }
                if(current_state.armed==false){
                    std::cout<<"not armed!!!"<<std::endl;
                    break;
                }
                uav_state=UAV_LANDING_START;
                std::cout<<"auto landing start"<<std::endl;
                break;
            default:break;
        }
        #endif
        
        // std::cout<<"LocalPosition:"<<std::setw(10)<<uavCurrentLocalPose.pose.position.x<<","<<std::setw(10)
        //     <<uavCurrentLocalPose.pose.position.y<<","<<std::setw(10)<<uavCurrentLocalPose.pose.position.z<<std::endl;

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}