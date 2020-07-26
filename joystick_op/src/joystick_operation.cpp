/**
 * @file joystick_operation.cpp
 * @author MJY1996 (1977415395@qq.com)
 * @brief decode the joystick message to control the car
 * @version 0.1
 * @date 2020-07-24
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include <ros/ros.h>
#include <driver_ctrl/ctrl_msg.h>
#include <sensor_msgs/Joy.h>

class JoystickOp
{
public:
    JoystickOp();
    void JoystickCountDown();
    bool JoystickConnected();

private:
    ros::NodeHandle _nh;
    ros::Subscriber _joy_subscriber;    // Receive the messages from the joystick
    ros::ServiceClient _joy_controller; // Send control message to the communication node
    driver_ctrl::ctrl_msg _control_msg;
    uint8_t _connect_count_down; // Be set 10 on receiving joystick message and count down once every 20ms.
                                 // Being equal to 0 indicats that the joystick is disconnected

    void JoyStickMsgHandler(const sensor_msgs::Joy::ConstPtr &joy);
};

/**
 * @brief Construct a new Joystick Op:: Joystick Op object
 * 
 */
JoystickOp::JoystickOp()
{
    _joy_subscriber = _nh.subscribe("joy", 1, &JoystickOp::JoyStickMsgHandler, this);
    _joy_controller = _nh.serviceClient<driver_ctrl::ctrl_msg>("control_msg");
}

/**
 * @brief Being called every 20ms to refresh the count down varible.
 *        The count down varible being equal to 0 means that the joystick is disconnected.
 * 
 */
void JoystickOp::JoystickCountDown()
{
    if (_connect_count_down > 0)
    {
        _connect_count_down--;
        if (_connect_count_down == 0)
        {
            // joystick disconnected error handler
            ROS_INFO("Joystick disconnected");
        }
    }
}

/**
 * @brief To check whether the joystick is connected
 * 
 * @return true The joystick is connected
 * @return false The joystick is disconnected
 */
bool JoystickOp::JoystickConnected()
{
    if (_connect_count_down > 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief decode the message from the joystick and control the car and reset the count down
 * 
 * @param joy 
 */
void JoystickOp::JoyStickMsgHandler(const sensor_msgs::Joy::ConstPtr &joy)
{
    _connect_count_down = 10;
    _control_msg.request.angular_velocity = joy->axes[0] * 30;
    _control_msg.request.speed = 1 - (joy->axes[5] + 1) / 2;
    if (joy->buttons[3] == 1)
    {
        _control_msg.request.speed *= (-1);
    }
    if (!_joy_controller.call(_control_msg))
    {
        ROS_ERROR("Failed to call car controller");
    }
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
    ros::init(argc, argv, "joystick_operation");
    JoystickOp joystick_op;
    ros::Rate looprate(50);

    while (ros::ok())
    {
        joystick_op.JoystickCountDown();
        ros::spinOnce();
        looprate.sleep();
    }
    return 0;
}