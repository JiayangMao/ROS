#include <ros/ros.h>
#include <topic_demo/dot.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle nh;
    topic_demo::dot msg;
    msg.x = 1.0f;
    msg.y = 1.0f;
    msg.state = "working";
    ros::Publisher pub = nh.advertise<topic_demo::dot>("dot_info", 1);
    ros::Rate loop_rate(1.0f);
    while(ros::ok())
    {
        msg.x *= 1.01;
        msg.y *= 1.02;
        ROS_INFO("Talker:: DOT: x = %f, y = %f\n", msg.x, msg.y);
        pub.publish(msg);
        loop_rate.sleep();
    }
    return 0;
}