#!/usr/bin/env python

import rospy
import serial
from sensor_msgs.msg import NavSatFix

class GPS_reader:
    def __init__(self):
        rospy.init_node("gps_reader", anonymous=True)
        self.pub = rospy.Publisher(
            "gps_data", NavSatFix, queue_size=1)
        self.ser = serial.Serial("/dev/ttyUSB1", 38400, timeout=1)
        self.nav_state = NavSatFix()
        '''
        self.nav_state.position_covariance_type = 
        self.nav_state.position_covariance = 
        '''
        self.nav_status_dict = {"0": self.nav_state.status.STATUS_FIX,
                                "1": self.nav_state.status.STATUS_SBAS_FIX,
                                "2": self.nav_state.status.STATUS_GBAS_FIX}

    def loop(self):
        while not rospy.is_shutdown():
            GPS_data = self.ser.readline()
            self.read_GPS_data(GPS_data)

    def read_GPS_data(self, data):
        if (data[3:6] != "GGA"):
            return False
        if (data[0] != "$" or data[-5] != "*"):
            return False
        sum_check = 0
        for c in data[1:-5]:
            sum_check ^= ord(c)
        if (int(data[-4:-2], 16) != sum_check):
            return False
        rospy.logdebug(data)
        data = data.split(",")
        try:
            self.nav_state.latitude = float(data[2][0:3]) + float(data[2][3:]) / 60
            self.nav_state.longitude = float(data[4][0:3]) + float(data[4][3:]) / 60
            self.nav_state.status.status = self.nav_state.status.STATUS_GBAS_FIX
            self.nav_state.altitude = float(data[9])
            if data[3] == "S":
                self.nav_state.latitude *= -1
            if data[5] == "W":
                self.nav_state.longitude *= -1
            if data[0][2] == "P":
                self.nav_state.status.service = self.nav_state.status.SERVICE_GPS
            else:
                self.nav_state.status.service = self.nav_state.status.SERVICE_COMPASS
            self.nav_state.header.stamp = rospy.Time.now()
            self.pub.publish(self.nav_state)
            return True
        except:
            return False


if __name__ == "__main__":
    gps = GPS_reader()
    gps.loop()
