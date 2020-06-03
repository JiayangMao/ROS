#include <ros/ros.h>
#include <serial/serial.h>

#include <driver_ctrl/fb_msg.h>
#include <driver_ctrl/ctrl_msg.h>

#include <stdint.h>

serial::Serial ser;

enum CtrlCmd
{
    cmd_car_state = 0x00,
    cmd_speed_ctrl = 0x20,
    cmd_gyro_ctrl = 0x21,
    cmd_set_odometry_l = 0x50,
    cmd_set_odometry_r = 0x51,
    cmd_set_yaw = 0x52,
};

enum CarStateCmd
{
    car_state_cmd_stop = 0x00,
    car_state_cmd_start = 0x11,
    car_state_cmd_heartbeat = 0x22,
};

const uint16_t MSG_HEAD = 0xeb90;
const size_t TX_SIZE = 8;
const size_t RX_SIZE = 47;

void CopyMsg(uint8_t *from, uint8_t *to, uint8_t size)
{
    while (size--)
    {
        to[size] = from[size];
    }
}

uint8_t CalcCheck(uint8_t *msg, uint8_t size)
{
    uint8_t sum = 0;
    while (size--)
    {
        sum += msg[size];
    }
    return sum;
}

bool CheckFormat(uint8_t *msg, uint8_t size)
{
    if ((msg[0] != static_cast<uint8_t>(MSG_HEAD >> 8)) ||
        (msg[1] != static_cast<uint8_t>(MSG_HEAD & 0x00ff)))
    {
        return false;
    }
    if (CalcCheck(msg, size - 1) != msg[size - 1])
    {
        return false;
    }
    else
    {
        return true;
    }
}

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

bool serial_tx_handle_function(driver_ctrl::ctrl_msg::Request &req,
                               driver_ctrl::ctrl_msg::Response &res)
{
    TxCmd(cmd_speed_ctrl, req.speed);
    TxCmd(cmd_gyro_ctrl, req.angular_velocity);
    return true;
}

bool serial_rx_function(driver_ctrl::fb_msg  &msg)
{
    size_t n = ser.available();
    if (n == 0)
    {
        return false;
    }
    uint8_t buffer[100];
    n = ser.read(buffer, n);
    if (n != RX_SIZE)
    {
        return false;
    }
    if (CheckFormat(buffer, n - 1) == false)
    {
        return false;
    }
    union TmpData
    {
        float f32;
        uint8_t u8[4];
    } tmp;
    size_t i = 2;
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
    return true;
}

// bool serial_rx_function(serial_demo::serial_rx &msg)
// {
//     size_t n = ser.available();
//     if (n == 0)
//     {
//         return false;
//     }
//     else
//     {
//         uint8_t buffer[10];
//         n = ser.read(buffer, n);
//         if (n == 7) 
//         {
//             msg.x_array = buffer[2];
//             msg.y_array = buffer[3];
//             // msg.buttom_a = (buffer[4]) && 0x01;
//             // msg.buttom_b = (buffer[4] >> 1) && 0x01;
//             // msg.buttom_c = (buffer[4] >> 2) && 0x01;
//             return true;
//         }
//         else
//         {
//             ROS_INFO("message size = %d", static_cast<int>(n));
//             return false;
//         }
//     }
//     return true;
// }

int main(int argc, char** argv)
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

    TxCmd(cmd_car_state, car_state_cmd_start);
    driver_ctrl::fb_msg feedback_msg;
    ros::Publisher ser_rx = nh.advertise<driver_ctrl::fb_msg>("feedback_msg", 1);
    ros::ServiceServer ser_tx = nh.advertiseService("control_msg", serial_tx_handle_function);
    ros::Rate looprate(100);

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
