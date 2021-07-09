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
        rospy.init_node('position_controller')  # initializing ros node with name position_controller

        #drone_command message format
        self.cmd_drone = edrone_cmd()
        self.cmd_drone.rcRoll = 1500
        self.cmd_drone.rcPitch = 1500
        self.cmd_drone.rcYaw = 1500
        self.cmd_drone.rcThrottle = 0

        #coordinates_error message format
        #self.coordinates_error = NavSatFix()
        #self.coordinates_error.latitude = 0
        #self.coordinates_error.longitude = 0
        #self.coordinates_error.altitude = 0

        #initial coordinates of the drone
        self.coordinates = [19.0009248718, 71.9998318945, 22.16]
        #self.coordinates = [19.0, 72.0, 0.31]
        #checkpoints
        self.checkpoint = [19.0009248718, 71.9998318945, 22.16]
        #self.checkpoint1 = [19.0009248718, 71.9998318945, 25.16]
        #self.checkpoint2 = [19.0007046575, 71.9998955286, 25.16]
        #self.checkpoint3 = [19.0007046575, 71.9998955286, 22.16]
        #self.checkpoint = [19.0, 72.0, 0.31]
        #self.checkpoint1 = [19.0, 72.0, 3.0]
        #self.checkpoint2 = [19.0000451704, 72.0, 3.0]
        #self.checkpoint3 = [19.0000451704, 72.0, 0.31]

        #flags
        self.flag1 = 0
        self.flag2 = 0

        self.Kp = [100000, 100000, 32.1]
        #self.Kp = [300000, 300000, 32.1]
        self.Ki = [0, 0, 0.11]
        self.Kd = [420000, 420000, 68.7]
        #self.Kd = [530000, 530000, 68.7]

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
        self.yaw_angle = 0

        self.sample_time = 0.050 #in seconds


        #publishing to the following topics
        self.cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
        #self.coordinates_error_pub = rospy.Publisher('coordinates_error', NavSatFix, queue_size = 1)
        #self.altitude_error_pub = rospy.Publisher('/altitude_error', Float32, queue_size=1)
        #self.latitude_error_pub = rospy.Publisher('/latitude_error', Float32, queue_size=1)
        #self.longitude_error_pub = rospy.Publisher('/longitude_error', Float32, queue_size=1)

        #subscribing to the following topics
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('my_topic', NavSatFix, self.checkpoint_callback)
        #rospy.Subscriber('/yaw_angle', Float32, self.yaw_angle_callback)
        #rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_set_pid)

    def gps_callback(self, msg):
        self.coordinates[0] = msg.latitude
        self.coordinates[1] = msg.longitude
        self.coordinates[2] = msg.altitude

    def checkpoint_callback(self, checkpoint_msg):
        self.checkpoint[0] = checkpoint_msg.latitude
        self.checkpoint[1] = checkpoint_msg.longitude
        self.checkpoint[2] = checkpoint_msg.altitude

    #def yaw_angle_callback(self, yaw_msg):
        #self.yaw_angle = yaw_msg

    #def altitude_set_pid(self, altitude_tune):
        #self.Kp[0] = altitude_tune.Kp * 150
        #self.Ki[0] = altitude_tune.Ki * 1
        #self.Kd[0] = altitude_tune.Kd * 200

    def pid(self):

        #if self.flag1 == 1 and self.flag2 == 0:
            #for i in range(3):
                #self.checkpoint[i] = self.checkpoint2[i]

        #elif self.flag1 == 1 and self.flag2 == 1:
            #for i in range(3):
                #self.checkpoint[i] = self.checkpoint3[i]

        #else:
            #for i in range(3):
                #self.checkpoint[i] = self.checkpoint1[i]

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

        #if(self.coordinates[2] < 25.26 and self.coordinates[2] > 25.06):
            #self.flag1 = 1
        #if(self.coordinates[0] < 19.0007091745 and self.coordinates[0] > 19.0007001405):
            #if(self.coordinates[1] < 71.9999002773 and self.coordinates[1] > 71.9998907799):
                #self.flag2 = 1
        #if(self.coordinates[2] < 3.05 and self.coordinates[2] > 2.95):
            #self.flag1 = 1
        #if(self.coordinates[0] < 19.000046 and self.coordinates[0] > 19.000045):
            #self.flag2 = 1
        #self.coordinates_error.latitude = self.error[0]
        #self.coordinates_error.longitude = self.error[1]
        #self.coordinates_error.altitude = self.error[2]


        self.cmd_pub.publish(self.cmd_drone)
        #self.latitude_error_pub.publish(self.error[0])
        #self.altitude_error_pub.publish(self.error[2])
        #self.longitude_error_pub.publish(self.error[1])
        #self.coordinates_error_pub.publish(self.coordinates_error)
        #print("Latitude error: ", self.error[0], "Longitude error: ", round(self.error[1], 3), "Altitude error: ", self.error[2])
        #print("Altitude: ", self.coordinates[2], "Altitude error: ", self.error[2], "Checkpoint", self.checkpoint[2])

if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(1/e_drone.sample_time)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()



        

