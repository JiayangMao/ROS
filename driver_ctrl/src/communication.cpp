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

#include <driver_ctrl/fb_msg.h>
#include <driver_ctrl/ctrl_msg.h>

#include <stdint.h>

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
    TxCmd(cmd_speed_ctrl, req.speed);
    TxCmd(cmd_gyro_ctrl, req.angular_velocity);
    return true;
}

uint8_t buffer[3 * RX_SIZE];
uint16_t end_flag = 0;

/**
 * @brief decode datas received from the controller, if there is a complete
 *        set of data in buffer, wite data in message and return true
 * 
 * @param msg pointer of the message to be released,
 *            including state feedback of the tricycle
 * @return true if the buffer contains a complete set of data
 * @return false 
 */
bool serial_rx_function(driver_ctrl::fb_msg &msg)
{
    size_t n = ser.available();
    if (n == 0)
    {
        return false;
    }
    n = ser.read(buffer + end_flag, n);
    end_flag += n;
    uint16_t head_flag = 0;
    while ((end_flag - head_flag) >= RX_SIZE)
    {
        if (CheckFormat(buffer + head_flag, RX_SIZE))
        {
            union TmpData
            {
                float f32;
                uint8_t u8[4];
            } tmp;
            uint16_t i = 2;
            CopyMsg(buffer + i, tmp.u8, 4);
            msg.speed_l = tmp.f32;
            i += 4;
            CopyMsg(buffer + i, tmp.u8, 4);
            msg.speed_r = tmp.f32;
            i += 4;
            CopyMsg(buffer + i, tmp.u8, 4);
            msg.odometry_l = tmp.f32;
            i += 4;
            CopyMsg(buffer + i, tmp.u8, 4);
            msg.odometry_r = tmp.f32;
            i += 4;
            CopyMsg(buffer + i, tmp.u8, 4);
            msg.gyro_x = tmp.f32;
            i += 4;
            CopyMsg(buffer + i, tmp.u8, 4);
            msg.gyro_y = tmp.f32;
            i += 4;
            CopyMsg(buffer + i, tmp.u8, 4);
            msg.gyro_z = tmp.f32;
            i += 4;
            CopyMsg(buffer + i, tmp.u8, 4);
            msg.accel_x = tmp.f32;
            i += 4;
            CopyMsg(buffer + i, tmp.u8, 4);
            msg.accel_y = tmp.f32;
            i += 4;
            CopyMsg(buffer + i, tmp.u8, 4);
            msg.accel_z = tmp.f32;
            i += 4;
            CopyMsg(buffer + i, tmp.u8, 4);
            msg.vin = tmp.f32;
            for (i = 0; i < (end_flag - head_flag - RX_SIZE); i++)
            {
                buffer[i] = buffer[head_flag + RX_SIZE];
            }
            end_flag = end_flag - head_flag - RX_SIZE;
            return true;
        }
        else
        {
            head_flag++;
        }
    }
    for (int i = 0; i < (end_flag - head_flag); i++)
    {
        buffer[i] = buffer[head_flag];
    }
    end_flag = end_flag - head_flag;
    return false;
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

    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
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

    TxCmd(cmd_car_state, car_state_cmd_start);
    driver_ctrl::fb_msg feedback_msg;
    ros::Publisher ser_rx = nh.advertise<driver_ctrl::fb_msg>("feedback_msg", 1);
    ros::ServiceServer ser_tx = nh.advertiseService("control_msg", serial_tx_handle_function);
    ros::Rate looprate(50);

    while (ros::ok())
    {
        ros::spinOnce();
        TxCmd(cmd_car_state, car_state_cmd_heartbeat);
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
