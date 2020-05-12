#include <ros/ros.h>
#include <serial_demo/serial_rx.h>

void serial_rx_handle_function(const serial_demo::serial_rx::ConstPtr &msg)
{
    ROS_INFO("Serial message: x = %d, y = %d", msg->x_array, msg->y_array);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "receiver");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("rx_msg", 1, serial_rx_handle_function);
    ros::spin();
    return 0;
}