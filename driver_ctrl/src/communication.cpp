/**
 * @file communication.cpp
 * @author MJY1996 (1977415395@qq.com)
 * @brief serial communication node for tricycle controller
 * @version 0.1
 * @date 2020-07-06
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>

#include <driver_ctrl/fb_msg.h>
#include <driver_ctrl/ctrl_msg.h>

#include <stdint.h>
#include <string>

/**
 * @brief serial port for communicating with tricycle
 * 
 */
serial::Serial ser;

/**
 * @brief command key
 * 
 */
enum CtrlCmd
{
    cmd_car_state = 0x00,      // control tricycle on or off
    cmd_speed_ctrl = 0x20,     // control linear velocity
    cmd_gyro_ctrl = 0x21,      // control angular velocity
    cmd_set_odometry_l = 0x50, // set odometry value of left wheel
    cmd_set_odometry_r = 0x51, // set odometry value of right wheel
    cmd_set_yaw = 0x52,        // set yaw angle
};

/**
 * @brief control value for command kye cmd_car_state
 * 
 */
enum CarStateCmd
{
    car_state_cmd_stop = 0x00,      // turn on the controller
    car_state_cmd_start = 0x11,     // turn off the controller
    car_state_cmd_heartbeat = 0x22, // periodically send heartbeat data
                                    // incase the controller turns off after
                                    // 1 second receive no datas
};

const uint16_t MSG_HEAD = 0xeb90; // head word of data
const size_t TX_SIZE = 8;         // size of data sending to controller
const size_t RX_SIZE = 47;        // size of data receiving from controller

/**
 * @brief copy data
 * 
 * @param from source address
 * @param to target address
 * @param size size of data
 */
void CopyMsg(uint8_t *from, uint8_t *to, uint8_t size)
{
    while (size--)
    {
        to[size] = from[size];
    }
}

/**
 * @brief split a string to a list
 * 
 * @param s origin string to be splited
 * @param v target string vector to keep the result
 * @param c the separator to use when splitting the string
 */
void SplitString(const std::string &s,
                 std::vector<std::string> &v,
                 const std::string &c)
{
    std::string::size_type pos1, pos2;
    pos2 = s.find(c);
    pos1 = 0;
    while (std::string::npos != pos2)
    {
        v.push_back(s.substr(pos1, pos2 - pos1));

        pos1 = pos2 + c.size();
        pos2 = s.find(c, pos1);
    }
    if (pos1 != s.length())
        v.push_back(s.substr(pos1));
}

/**
 * @brief calculate the check byte using sum check
 * 
 * @param msg data to be checked
 * @param size size of data
 * @return uint8_t sum check byte
 */
uint8_t CalcCheck(uint8_t *msg, uint8_t size)
{
    uint8_t sum = 0;
    while (size--)
    {
        sum += msg[size];
    }
    return sum;
}

/**
 * @brief check if the datas conforms to the format
 *        according to its header and check byte
 * 
 * @param msg datas to be checked
 * @param size size of datas
 * @return true the datas conforms to the format
 * @return false the datas coes not conform to the format
 */
bool CheckFormat(uint8_t *msg, uint8_t size)
{
    if ((msg[0] != static_cast<uint8_t>(MSG_HEAD >> 8)) ||
        (msg[1] != static_cast<uint8_t>(MSG_HEAD & 0x00ff)))
    {
        // ROS_INFO_STREAM("head error");
        return false;
    }
    if (CalcCheck(msg, size - 1) != msg[size - 1])
    {
        return false;
    }
    else
    {
        // ROS_INFO_STREAM("check error");
        return true;
    }
}

/**
 * @brief send control command
 * 
 * @param cmd key of command
 * @param data value of command
 */
void TxCmd(CtrlCmd cmd, float data)
{
    uint8_t tx_data[TX_SIZE];
    tx_data[0] = static_cast<uint8_t>(MSG_HEAD >> 8);
    tx_data[1] = static_cast<uint8_t>(MSG_HEAD & 0xff);
    tx_data[2] = static_cast<uint8_t>(cmd);
    CopyMsg(reinterpret_cast<uint8_t *>(&data), tx_data + 3, 4);
    tx_data[TX_SIZE - 1] = CalcCheck(tx_data, TX_SIZE - 1);
    ser.write(tx_data, TX_SIZE);
}

/**
 * @brief send command to control car state
 * 
 * @param cmd key of commane
 * @param state value of command
 */
void TxCmd(CtrlCmd cmd, CarStateCmd state)
{
    uint8_t tx_data[TX_SIZE];
    tx_data[0] = static_cast<uint8_t>(MSG_HEAD >> 8);
    tx_data[1] = static_cast<uint8_t>(MSG_HEAD & 0xff);
    tx_data[2] = static_cast<uint8_t>(cmd);
    tx_data[3] = static_cast<uint8_t>(state);
    tx_data[TX_SIZE - 1] = CalcCheck(tx_data, TX_SIZE - 1);
    ser.write(tx_data, TX_SIZE);
}

float linear_velocity, angular_velocity;
int out_time = 0;
/**
 * @brief control car according to the subscribed service
 * 
 * @param req request of the service, including 
 *            linear velocity and angular velocity
 * @param res response of the service, null for this service
 * @return true successfully handle the request
 * @return false 
 */
bool serial_tx_handle_function(driver_ctrl::ctrl_msg::Request &req,
                               driver_ctrl::ctrl_msg::Response &res)
{
    out_time = 1000;
    linear_velocity = req.speed;
    angular_velocity = req.angular_velocity;
    return true;
}

uint8_t buffer[3 * RX_SIZE];
uint16_t end_flag = 0;

/**
 * @brief decode datas received from the controller,
 *        if there is complete, wite data in message and return true
 * 
 * @param msg pointer of the message to be released,
 *            including state feedback of the tricycle
 * @return true if the data received is complete
 * @return false 
 */
bool serial_rx_function(driver_ctrl::fb_msg &msg)
{
    std::string rx_data;
    size_t rx_size = ser.readline(rx_data);
    if (rx_data[0] != '#' || rx_data[rx_size - 2] != '*' || rx_size > 100)
    {
        ROS_INFO("FORMAT ERROR: %s", rx_data.c_str());
        return false;
    }
    rx_data.erase(rx_data.begin());
    std::vector<std::string> data_list;
    SplitString(rx_data, data_list, ",");
    if (data_list.size() != 9)
    {
        ROS_INFO("DATA SIZE ERROR: %d", static_cast<int>(data_list.size()));
        return false;
    }
    msg.speed_l = std::stof(data_list[0]);
    msg.speed_r = std::stof(data_list[1]);
    msg.odometry_l = std::stof(data_list[2]);
    msg.odometry_r = std::stof(data_list[3]);
    msg.roll = std::stof(data_list[4]);
    msg.pitch = std::stof(data_list[5]);
    msg.yaw = std::stof(data_list[6]);
    msg.angular_velocity = std::stof(data_list[7]);

    

    return true;
}

/**
 * @brief 
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "communication");
    ros::NodeHandle nh;

    std::string port;
    int baudrate;
    float control_rate;
    nh.param<std::string>("port", port, "/dev/ttyUSB0");
    nh.param<int>("baudrate", baudrate, 115200);
    nh.param<float>("control_rate", control_rate, 20.0f);
    linear_velocity = 0;
    angular_velocity = 0;
    try
    {
        ser.setPort(port);
        ser.setBaudrate(baudrate);
        serial::Timeout t = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(t);
        ser.open();
    }
    catch (serial::IOException &e)
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

    driver_ctrl::fb_msg feedback_msg;
    ros::Publisher ser_rx = nh.advertise<driver_ctrl::fb_msg>("feedback_msg", 1);
    ros::ServiceServer ser_tx = nh.advertiseService("control_msg", serial_tx_handle_function);
    ros::Rate looprate(control_rate);

    while (ros::ok())
    {
        ros::spinOnce();
        if (out_time > 0)
        {
            out_time -= static_cast<float>(1000.0 / control_rate);
        }
        else
        {
            linear_velocity = 0;
            angular_velocity = 0;
        }

        TxCmd(cmd_speed_ctrl, linear_velocity);
        TxCmd(cmd_gyro_ctrl, angular_velocity);
        if (serial_rx_function(feedback_msg))
        {
            ser_rx.publish(feedback_msg);
        }
        // if (serial_rx_function(rx_msg))
        // {
        //     ser_rx.publish(rx_msg);
        // }
        looprate.sleep();
    }
    return 0;
}
