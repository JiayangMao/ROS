#!/usr/bin/env python

import rospy
import socket

from driver_ctrl.srv import ctrl_msg


class car_client:
    def __init__(self):
        rospy.init_node("server_communication", anonymous=True)
        self.server_ip = rospy.get_param("~server_ip", "192.168.1.100")
        # self.server_ip = str(rospy.get_param("~server_ip", "101.132.135.141"))
        self.server_port = int(rospy.get_param("~server_port", 31415))
        self.report_rate = int(rospy.get_param("~report_rate", 10))
        rospy.loginfo(str(self.server_ip))
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.connect((self.server_ip, self.server_port))
        self.car_ctrl = rospy.ServiceProxy("control_msg", ctrl_msg)
        self.rate = rospy.Rate(self.report_rate)

    def __del__(self):
        try:
            rospy.loginfo("connection closed")
            self.client.close()
        except:
            pass

    def tx_msg(self, msg):
        self.client.send(msg.encode("utf-8"))

    def rx_msg(self):
        try:
            data = self.client.recv(100).decode("utf-8")
            return data.split("\n")[-2]
        except:
            return None

    def main_loop(self):
        while not rospy.is_shutdown():
            # receive message
            ctrl_msg = self.rx_msg()
            if ctrl_msg is not None:
                try:
                    ctrl_msg = ctrl_msg.split(",")
                    linear_velocity = float(ctrl_msg[0]) * 0.5
                    angular_velocity = float(ctrl_msg[1]) * (-80)
                    if not self.car_ctrl(linear_velocity, angular_velocity):
                        rospy.loginfo("failed to call controller")
                except:
                    rospy.logerr("remote control error")
                    pass

            # send message

            # self.rate.sleep()


if __name__ == "__main__":
    cc = car_client()
    cc.main_loop()
