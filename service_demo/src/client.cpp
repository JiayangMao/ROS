#include <ros/ros.h>
#include <service_demo/dot.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "client");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<service_demo::dot>("dot");

    service_demo::dot srv;
    srv.request.x = 0;
    srv.request.y = 0;

    ros::Rate loop_rate(1.0f);
    while (ros::ok())
    {
        srv.request.x++;
        srv.request.y++;
        if (client.call(srv))
        {
            ROS_INFO("radius = %f", srv.response.r);
        }
        else
        {
            ROS_INFO("Failed to call service");
        }
        loop_rate.sleep();
    }
    return 0;
}