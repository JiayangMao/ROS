#include <ros/ros.h>
#include <service_demo/dot.h>

bool handle_function(service_demo::dot::Request &req, service_demo::dot::Response &res)
{
    ROS_INFO("Request from dot: x = %f, y = %f", req.x, req.y);
    res.r = sqrt(pow(req.x, 2) + pow(req.y, 2));
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dot_server");
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("dot", handle_function);
    ros::spin();

    return 0;
}