#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"//这引用了 std_msgs/String 消息, 它存放在 std_msgs package 里，是由 String.msg 文件自动生成的头文件
// %EndTag(MSG_HEADER)%

#include <sstream>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");//初始化 ROS 。它允许 ROS 通过命令行进行名称重映射

  ros::NodeHandle n;//为这个进程的节点创建一个句柄。第一个创建的 NodeHandle 会为节点进行初始化，最后一个销毁的 NodeHandle 则会释放该节点所占用的所有资源

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
//告诉 master 我们将要在 chatter（话题名） 上发布 std_msgs/String 消息类型的消息。这样 master 就会告诉所有订阅了 chatter 话题的节点，将要有数据发布。第二个参数是发布序列的大小。如果我们发布的消息的频率太高，缓冲区中的消息在大于 1000 个的时候就会开始丢弃先前发布的消息
//NodeHandle::advertise() 返回一个 ros::Publisher 对象,它有两个作用： 1) 它有一个 publish() 成员函数可以让你在topic上发布消息； 2) 如果消息类型不对,它会拒绝发布



  ros::Rate loop_rate(10);//以 10Hz 的频率运行,ros::Rate 对象可以允许你指定自循环的频率。它会追踪记录自上一次调用 Rate::sleep() 后时间的流逝，并休眠直到一个频率周期的时间


  int count = 0;
  while (ros::ok())
  {

    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

    ++count;
  }


  return 0;
}


