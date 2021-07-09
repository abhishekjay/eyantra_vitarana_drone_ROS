#!/usr/bin/env python

from vitarana_drone.msg import *
from sensor_msgs.msg import LaserScan, NavSatFix, Image
from pyzbar.pyzbar import decode
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32, String
from vitarana_drone.srv import *
import cv2
import numpy as np
import rospy
import time
import tf
import math

class Pathplanner():

    def __init__(self):
        rospy.init_node('path_planner')

#------------------------------------------------
#message format

        self.target_coordinates = NavSatFix()
        self.target_coordinates.latitude = 19.0009248718
        self.target_coordinates.longitude = 71.9998318945
        self.target_coordinates.altitude = 22.16

#------------------------------------------------
#variables used

        self.coordinates = [19.0009248718, 71.9998318945, 22.16]
        self.checkpoint_latitude = [19.0009248718, 19.0007046575, 19.0007046575, 19.0007046575]
        self.checkpoint_longitude = [71.9998318945, 71.9998955286, 71.9998955286, 71.9998955286]
        self.checkpoint_altitude = [25.16, 25.16, 22.14, 25.16]

        self.coordinates = [0.0, 0.0, 0.0]
        self.deimg = ""
        self.split_img = []
        self.gripper_state = False

        self.goal_coordinates = [0, 0, 0]
        self.sensor_top = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.coordinate_error = [0.0, 0.0, 0.0]
        self.threshold = [0.000001, 0.05]

        self.i = 0
        self.x = 0
        self.y = 0
        self.lng = 0
        self.ltt = 0
        self.goal_x = 0
        self.goal_y = 0
        self.x_update = 0
        self.y_update = 0
        self.gripper_call = False
        self.goal_reached_flag = 0

        self.sample_time = 0.060  #in seconds

#------------------------------------------------
#subscribing to

        rospy.Subscriber('/edrone/range_finder_top', LaserScan, self.range_top_callback)
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/qr_data', String, self.qr_detect_callback)
        rospy.Subscriber('/edrone/gripper_check', String, self.gripper_callback)

#------------------------------------------------
#publishing to

        self.coordinates_pub = rospy.Publisher('my_topic', NavSatFix, queue_size=1)

#------------------------------------------------
#callback functions

    def gps_callback(self, msg):
        self.coordinates[0] = msg.latitude
        self.coordinates[1] = msg.longitude
        self.coordinates[2] = msg.altitude

    def range_top_callback(self, range_msg):
        self.sensor_top[0] = range_msg.ranges[0]
        self.sensor_top[1] = range_msg.ranges[1]
        self.sensor_top[2] = range_msg.ranges[2]
        self.sensor_top[3] = range_msg.ranges[3]
        self.sensor_top[4] = range_msg.ranges[4]

    def qr_detect_callback(self, qr_msg):
        self.deimg = str(qr_msg)
        self.deimg = self.deimg.strip('dat: "')
        self.split_img = self.deimg.split(',')
        self.goal_coordinates[0] = float(self.split_img[0])
        self.goal_coordinates[1] = float(self.split_img[1])
        self.goal_coordinates[2] = float(self.split_img[2])

    def gripper_callback(self, gripper_msg):
        print(gripper_msg)

#------------------------------------------------
#follow checkpoints from the arrays checkpoint_latitude, checkpoint_longitude and checkpoint_altitude.

    def navigate_to_checkpoint(self):

        self.target_coordinates.latitude = self.checkpoint_latitude[self.i]
        self.target_coordinates.longitude = self.checkpoint_longitude[self.i]
        self.target_coordinates.altitude = self.checkpoint_altitude[self.i]
        self.coordinates_pub.publish(self.target_coordinates)

        self.coordinate_error[0] = self.checkpoint_latitude[self.i] - self.coordinates[0]
        self.coordinate_error[1] = self.checkpoint_longitude[self.i] - self.coordinates[1]
        self.coordinate_error[2] = self.checkpoint_altitude[self.i] - self.coordinates[2]

        if(self.coordinate_error[0] > -self.threshold[0] and self.coordinate_error[0] < self.threshold[0]):
            if(self.coordinate_error[1] > -self.threshold[0] and self.coordinate_error[1] < self.threshold[0]):
                if(self.coordinate_error[2] > -self.threshold[1] and self.coordinate_error[2] < self.threshold[1]):
                    if(self.i > 2):
                        self.bug0()
                    self.i += 1
#request to grip the package on the 3rd checkpoint
        if(self.i == 3):
            rospy.sleep(3)
            self.gripper_control(True)
            rospy.sleep(3)
#request to detach the package at goal
        if(self.goal_reached_flag == 1):
            if ((self.coordinates[2] - self.goal_coordinates[2])**2)**0.5 < 0.1:
                rospy.sleep(2)
                self.gripper_control(False)
                rospy.sleep(2)

#------------------------------------------------------
#gripper control function, sends service message to '/edrone/activate_gripper' topic when called.

    def gripper_control(self, gripper_state):
        rospy.wait_for_service('/edrone/activate_gripper')
        try:
            box_pick = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
            result_ = box_pick(gripper_state)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

#------------------------------------------------------
#line to goal function. Returns suitable update_y coordinate for an input update_x, such that the point (update_x, update_y) lies in line with the goal.

    def line_to_goal(self, input_x, input_y, goal_x, goal_y, update_x):
        self.update_y = ((goal_y - input_y)/(goal_x - input_x))*(update_x - input_x) + input_y
        return self.update_y

#------------------------------------------------------
#coordinate conversions

    def latitude_to_x(self, input_latitude):
        return 110692.0702932625 * (input_latitude - 19)

    def longitude_to_y(self, input_longitude):
        return -105292.0089353767 * (input_longitude - 72)

    def x_to_latitude(self, input_x):
        return (input_x/110692.0702932625) + 19

    def y_to_longitude(self, input_y):
        return 72 - (input_y/105292.0089353767)

#-------------------------------------------------------
#obstacle avoidance algorithm. This is the bug0 algorithm that always takes left when obstacle is detected in front (within 15m), 
#follows the boundary of the obstacle till goal is in sight. After passing the obstacle, it moves in the direction to goal.

    def bug0(self):
#if obstacle is in the roll direction of the drone
        if(self.sensor_top[3] < 15):

            if(self.sensor_top[2] >= 25):
                if(self.sensor_top[3] > 5):
                    self.x = self.latitude_to_x(self.checkpoint_latitude[self.i])
                    self.ltt = self.x_to_latitude(self.x - 1)
                else:
                    self.x = self.latitude_to_x(self.checkpoint_latitude[self.i])
                    self.ltt = self.x_to_latitude(self.x + 1)
                #move left
                self.y = self.longitude_to_y(self.checkpoint_longitude[self.i])
                self.lng = self.y_to_longitude(self.y - 5)        
                self.checkpoint_latitude.append(self.ltt)
                self.checkpoint_longitude.append(self.lng)
                self.checkpoint_altitude.append(25.16)
#if obstacle is in the pitch direction of the drone
        elif(self.sensor_top[0] < 15):

            if(self.sensor_top[3] >= 25):
                if(self.sensor_top[0] < 5):
                    self.y = self.longitude_to_y(self.checkpoint_longitude[self.i])
                    self.lng = self.y_to_longitude(self.y - 1)
                else:
                    self.y = self.longitude_to_y(self.checkpoint_longitude[self.i])
                    self.lng = self.y_to_longitude(self.y + 1)
                #move forward
                self.x = self.latitude_to_x(self.checkpoint_latitude[self.i])
                self.ltt = self.x_to_latitude(self.x - 5)        
                self.checkpoint_latitude.append(self.ltt)
                self.checkpoint_longitude.append(self.lng)
                self.checkpoint_altitude.append(25.16)

#if no obstacle is detected, coordinates are updated such that they lie in line with the goal

        else:
            if(self.goal_reached_flag == 0):
                self.x = self.latitude_to_x(self.checkpoint_latitude[self.i])
                self.y = self.longitude_to_y(self.checkpoint_longitude[self.i])
                self.x_update = self.x - 10
                self.ltt = self.x_to_latitude(self.x_update)
            
                self.goal_x = self.latitude_to_x(self.goal_coordinates[0])

                self.goal_y = self.longitude_to_y(self.goal_coordinates[1])

                self.y_update = self.line_to_goal(self.x, self.y, self.goal_x, self.goal_y, self.x_update)

                self.lng = self.y_to_longitude(self.y_update)

                self.checkpoint_latitude.append(self.ltt)
                self.checkpoint_longitude.append(self.lng)
                self.checkpoint_altitude.append(25.16)

#if goal is within 25m of drone, it will move to goal in two coordinate commands. Also raise the goal_reached_flag.

                if((self.goal_x - self.x)**2 + (self.goal_y - self.y)**2)**0.5 < 25:
                    self.checkpoint_latitude.append(self.goal_coordinates[0])
                    self.checkpoint_longitude.append(self.goal_coordinates[1])
                    self.checkpoint_altitude.append(25.16)
                    self.checkpoint_latitude.append(self.goal_coordinates[0])
                    self.checkpoint_longitude.append(self.goal_coordinates[1])
                    self.checkpoint_altitude.append(self.goal_coordinates[2])
                    self.goal_reached_flag = 1
                    self.checkpoint_latitude.append(self.goal_coordinates[0])
                    self.checkpoint_longitude.append(self.goal_coordinates[1])
                    self.checkpoint_altitude.append(self.goal_coordinates[2] + 3)


#---------------------------------------------------


if __name__ == '__main__':

    path_planner = Pathplanner()
    r = rospy.Rate(1/path_planner.sample_time)
    
    while not rospy.is_shutdown():
        #path_planner.bug2()
        path_planner.navigate_to_checkpoint()
        r.sleep()

   



        



