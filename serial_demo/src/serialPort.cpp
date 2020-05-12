#include <ros/ros.h>
#include <serial/serial.h>

#include <serial_demo/serial_rx.h>
#include <serial_demo/serial_tx.h>

#include <stdint.h>

serial::Serial ser;

bool serial_tx_handle_function(serial_demo::serial_tx::Request &req,
                               serial_demo::serial_tx::Response &res)
{
    ser.write(req.msg);
    return true;
}

bool serial_rx_function(serial_demo::serial_rx &msg)
{
    size_t n = ser.available();
    if (n == 0)
    {
        return false;
    }
    else
    {
        uint8_t buffer[10];
        n = ser.read(buffer, n);
        if (n == 7) 
        {
            msg.x_array = buffer[2];
            msg.y_array = buffer[3];
            // msg.buttom_a = (buffer[4]) && 0x01;
            // msg.buttom_b = (buffer[4] >> 1) && 0x01;
            // msg.buttom_c = (buffer[4] >> 2) && 0x01;
            return true;
        }
        else
        {
            ROS_INFO("message size = %d", static_cast<int>(n));
            return false;
        }
    }
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serialPort");
    ros::NodeHandle nh;

    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout t = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(t);
        ser.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Failed to open serial port");
        return -1;
    }
    if (ser.isOpen())
    {
        ROS_INFO("serial port opened");
    }
    else
    {
        return -1;
    }    

    serial_demo::serial_rx rx_msg;
    ros::Publisher ser_rx = nh.advertise<serial_demo::serial_rx>("rx_msg", 1);
    ros::ServiceServer ser_tx = nh.advertiseService("tx_msg", serial_tx_handle_function);
    ros::Rate looprate(500);
    while (ros::ok())
    {
        ros::spinOnce();
        if (serial_rx_function(rx_msg))
        {
            ROS_INFO("received a message");
            ser_rx.publish(rx_msg);
        }
    }
    return 0;
}