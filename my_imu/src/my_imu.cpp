#include <stdlib.h>
#include <cmath>

#include <ros/ros.h>
#include <driver_ctrl/fb_msg.h>
#include <sensor_msgs/Imu.h>

class IMU
{
public:
    IMU();
    void Update(double dt);
    void GetQuat(double &w, double &x, double &y, double &z);
    void SetGyro(double x, double y, double z);
    void SetAccel(double x, double y, double z);
    void GetGyro(double &x, double &y, double &z);
    void GetAccel(double &x, double &y, double &z);

private:
    double const Ki = 0.001f;
    double const Kp = 0.4f;

    double accel[3], gyro[3];
    double quat[4];
    double rMat[3][3];

    double exInt;
    double eyInt;
    double ezInt;
};

IMU::IMU()
{
    quat[0] = 1;
    quat[1] = 0;
    quat[2] = 0;
    quat[3] = 0;
}

void IMU::Update(double dt = 0.05)
{
    double norm;
    double ex, ey, ez;
    double halft;

    double ax, ay, az, gx, gy, gz;
    double _q0, _q1, _q2, _q3;

    double q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    double yaw;

    halft = 0.5 * dt;
    ax = this->accel[0];
    ay = this->accel[1];
    az = this->accel[2];
    gx = this->gyro[0];
    gy = this->gyro[1];
    gz = this->gyro[2];

    q0q1 = quat[0] * quat[1];
    q0q2 = quat[0] * quat[2];
    q0q3 = quat[0] * quat[3];
    q1q1 = quat[1] * quat[1];
    q1q2 = quat[1] * quat[2];
    q1q3 = quat[1] * quat[3];
    q2q2 = quat[2] * quat[2];
    q2q3 = quat[2] * quat[3];
    q3q3 = quat[3] * quat[3];

    if ((ax != 0.0f) || (ay != 0.0f) || (az != 0.0f))
    {
        norm = sqrtf64(ax * ax + ay * ay + az * az);
        ax /= norm;
        ay /= norm;
        az /= norm;

        ex = (ay * rMat[2][2] - az * rMat[2][1]);
        ey = (az * rMat[2][0] - ax * rMat[2][2]);
        ez = (ax * rMat[2][1] - ay * rMat[2][0]);

        exInt += Ki * ex * dt;
        eyInt += Ki * ey * dt;
        ezInt += Ki * ez * dt;

        gx += Kp * ex + exInt;
        gy += Kp * ey + eyInt;
        gz += Kp * ez + ezInt;
    }

    _q0 = quat[0];
    _q1 = quat[1];
    _q2 = quat[2];
    _q3 = quat[3];

    quat[0] += (-_q1 * gx - _q2 * gy - _q3 * gz) * halft;
    quat[1] += (_q0 * gx + _q2 * gz - _q3 * gy) * halft;
    quat[2] += (_q0 * gy - _q1 * gz + _q3 * gx) * halft;
    quat[3] += (_q0 * gz + _q1 * gy - _q2 * gx) * halft;

    norm = sqrtf64(quat[0] * quat[0] + quat[1] * quat[1] +
                   quat[2] * quat[2] + quat[3] * quat[3]);
    quat[0] /= norm;
    quat[1] /= norm;
    quat[2] /= norm;
    quat[3] /= norm;

    rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    rMat[0][1] = 2.0f * (q1q2 + -q0q3);
    rMat[0][2] = 2.0f * (q1q3 - -q0q2);

    rMat[1][0] = 2.0f * (q1q2 - -q0q3);
    rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    rMat[1][2] = 2.0f * (q2q3 + -q0q1);

    rMat[2][0] = 2.0f * (q1q3 + -q0q2);
    rMat[2][1] = 2.0f * (q2q3 - -q0q1);
    rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;

//     ROS_INFO("yaw = %f", atan2f64(rMat[1][0], rMat[0][0]) * 57.29578);
//     ROS_INFO("pitch = %f", -asinf64(rMat[2][0]) * 57.29578);
//     ROS_INFO("roll = %f", atan2f64(rMat[2][1], rMat[2][2]) * 57.29578);
}

void IMU::GetQuat(double &w, double &x, double &y, double &z)
{
    w = quat[0];
    x = quat[1];
    y = quat[2];
    z = quat[3];
}

void IMU::SetGyro(double x, double y, double z)
{
    gyro[0] = x;
    gyro[1] = y;
    gyro[2] = z;
}

void IMU::SetAccel(double x, double y, double z)
{
    accel[0] = x;
    accel[1] = y;
    accel[2] = z;
}

void IMU::GetGyro(double &x, double &y, double &z)
{
    x = gyro[0];
    y = gyro[1];
    z = gyro[2];
}

void IMU::GetAccel(double &x, double &y, double &z)
{
    x = accel[0];
    y = accel[1];
    z = accel[2];
}

class IMUNode
{
public:
    IMUNode();

    void FeedBackHandler(const driver_ctrl::fb_msg::ConstPtr &msg);

private:
    IMU imu;
    ros::NodeHandle nh;
    ros::Subscriber feedback_sub;
    ros::Publisher imu_pub;
    driver_ctrl::fb_msg feedback_msg;
    sensor_msgs::Imu imu_msg;
};

IMUNode::IMUNode()
{
    feedback_sub = nh.subscribe("feedback_msg", 1, &IMUNode::FeedBackHandler, this);
    imu_pub = nh.advertise<sensor_msgs::Imu>("imu_msg", 1);
}

void IMUNode::FeedBackHandler(const driver_ctrl::fb_msg::ConstPtr &msg)
{
    imu.SetGyro(msg->gyro_x, msg->gyro_y, msg->gyro_z);
    imu.SetAccel(msg->accel_x, msg->accel_y, msg->accel_z);
    imu.Update();
    imu_msg.header.stamp = ros::Time::now();
    imu.GetQuat(imu_msg.orientation.w, imu_msg.orientation.x,
                imu_msg.orientation.y, imu_msg.orientation.z);
    imu.GetGyro(imu_msg.angular_velocity.x,
                imu_msg.angular_velocity.y,
                imu_msg.angular_velocity.z);
    imu.GetAccel(imu_msg.linear_acceleration.x,
                 imu_msg.linear_acceleration.y,
                 imu_msg.linear_acceleration.z);
    imu_pub.publish(imu_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_imu");
    IMUNode my_imu;
    ros::spin();
    return 1;
}