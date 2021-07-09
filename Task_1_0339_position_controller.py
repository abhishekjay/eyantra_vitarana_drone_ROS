#!/usr/bin/env python

# Importing the required libraries

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32
import rospy
import time
import tf


class Edrone():
    """docstring for Edrone"""
    def __init__(self):
        rospy.init_node('position_controller1')  # initializing ros node with name position_controller1

        #drone_command message format
        self.cmd_drone = edrone_cmd()
        self.cmd_drone.rcRoll = 1500
        self.cmd_drone.rcPitch = 1500
        self.cmd_drone.rcYaw = 1500
        self.cmd_drone.rcThrottle = 0

        #initial coordinates of the drone
        self.coordinates = [19.0, 72.0, 0.31]

        #checkpoints
        self.checkpoint = [19.0, 72.0, 0.31]
        self.checkpoint1 = [19.0, 72.0, 3.0]
        self.checkpoint2 = [19.0000451704, 72.0, 3.0]
        self.checkpoint3 = [19.0000451704, 72.0, 0.31]

        #flags
        self.flag1 = 0
        self.flag2 = 0

        self.Kp = [294750, 294750, 32.1]
        self.Ki = [0, 0, 0.11]
        self.Kd = [707400, 707400, 68.7]

        self.error = [0, 0, 0]
        self.prev_error = [0, 0, 0]
        self.error_difference = [0, 0, 0]
        self.error_sum = [0, 0, 0]
        self.p_output = [0, 0, 0]
        self.i_output = [0, 0, 0]
        self.d_output = [0, 0, 0]
        self.pid_output = [0, 0, 0]
        self.max_value = [2000, 1024]
        self.min_value = [1000, 0]

        self.sample_time = 0.060 #in seconds


        #publishing to the following topics
        self.cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
        self.altitude_error_pub = rospy.Publisher('/altitude_error', Float32, queue_size=1)
        self.latitude_error_pub = rospy.Publisher('/latitude_error', Float32, queue_size=1)
        self.longitude_error_pub = rospy.Publisher('/longitude_error', Float32, queue_size=1)

        #subscribing to the following topics
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        #rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_set_pid)

    def gps_callback(self, msg):
        self.coordinates[0] = msg.latitude
        self.coordinates[1] = msg.longitude
        self.coordinates[2] = msg.altitude

    #def altitude_set_pid(self, altitude_tune):
        #self.Kp[0] = altitude_tune.Kp * 150
        #self.Ki[0] = altitude_tune.Ki * 1
        #self.Kd[0] = altitude_tune.Kd * 200

    def pid(self):

        if self.flag1 == 1 and self.flag2 == 0:
            for i in range(3):
                self.checkpoint[i] = self.checkpoint2[i]

        elif self.flag1 == 1 and self.flag2 == 1:
            for i in range(3):
                self.checkpoint[i] = self.checkpoint3[i]

        else:
            for i in range(3):
                self.checkpoint[i] = self.checkpoint1[i]

        self.error[0] = self.checkpoint[0] - self.coordinates[0]
        self.error_difference[0] = self.error[0] - self.prev_error[0]
        self.error_sum[0] = self.error_sum[0] + self.error[0]

        self.error[1] = self.checkpoint[1] - self.coordinates[1]
        self.error_difference[1] = self.error[1] - self.prev_error[1]
        self.error_sum[1] = self.error_sum[1] + self.error[1]

        self.error[2] = self.checkpoint[2] - self.coordinates[2]
        self.error_difference[2] = self.error[2] - self.prev_error[2]
        if self.error[2] > -0.15 and self.error[2] < 0.15:
            self.error_sum[2] = self.error_sum[2] + self.error[2]

        self.pid_output[0] = (self.error[0] * self.Kp[0]) + (self.error_sum[0] * self.Ki[0]) + (self.error_difference[0] * self.Kd[0]/self.sample_time)
        self.pid_output[1] = (self.error[1] * self.Kp[1]) + (self.error_sum[1] * self.Ki[1]) + (self.error_difference[1] * self.Kd[1]/self.sample_time)
        self.pid_output[2] = (self.error[2] * self.Kp[2]) + (self.error_sum[2] * self.Ki[2]) + (self.error_difference[2] * self.Kd[2]/self.sample_time)

        self.cmd_drone.rcRoll = 1500 + self.pid_output[0]
        self.cmd_drone.rcPitch = 1500 + self.pid_output[1]
        self.cmd_drone.rcThrottle = 512 + self.pid_output[2]

        if self.cmd_drone.rcRoll > self.max_value[0]:
            self.cmd_drone.rcRoll = self.max_value[0]
        elif self.cmd_drone.rcRoll < self.min_value[0]:
            self.cmd_drone.rcRoll = self.min_value[0]       

        if self.cmd_drone.rcPitch > self.max_value[0]:
            self.cmd_drone.rcPitch = self.max_value[0]
        elif self.cmd_drone.rcPitch < self.min_value[0]:
            self.cmd_drone.rcPitch = self.min_value[0]

        if self.cmd_drone.rcThrottle > self.max_value[1]:
            self.cmd_drone.rcThrottle = self.max_value[1]
        elif self.cmd_drone.rcThrottle < self.min_value[1]:
            self.cmd_drone.rcThrottle = self.min_value[1]

        self.prev_error[0] = self.error[0]
        self.prev_error[1] = self.error[1]
        self.prev_error[2] = self.error[2]

        if(self.coordinates[2] < 3.1 and self.coordinates[2] > 2.9):
            self.flag1 = 1
        if(self.coordinates[0] < 19.0000496874 and self.coordinates[0] > 19.0000406534):
            self.flag2 = 1


        self.cmd_pub.publish(self.cmd_drone)
        self.latitude_error_pub.publish(self.error[0])
        self.longitude_error_pub.publish(self.error[1])
        self.altitude_error_pub.publish(self.error[2])
        print("Latitude error: ", self.error[0], "Longitude error: ", round(self.error[1], 3), "Altitude error: ", self.error[2])
        print("Latitude: ", self.coordinates[0], "Longitude: ", round(self.coordinates[1], 3), "Altitude: ", self.coordinates[2])

if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(1/e_drone.sample_time)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()



        

