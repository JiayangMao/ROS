#include <ros/ros.h>
#include <serial_demo/serial_tx.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "transmitter");
    ros::NodeHandle nh;
    serial_demo::serial_tx tx_msg;
    ros::ServiceClient tx_srv = nh.serviceClient<serial_demo::serial_tx>("tx_msg");
    ros::Rate loop_rate(1);
    while(ros::ok())
    {
        tx_msg.request.msg = "Hello Word";
        tx_srv.call(tx_msg);
        loop_rate.sleep();
    }
    return 0;
}