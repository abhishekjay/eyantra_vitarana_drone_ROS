#!/usr/bin/env python


'''
This is a boiler plate script that contains an example on how to subscribe a rostopic containing camera frames 
and store it into an OpenCV image to use it further for image processing tasks.
Use this code snippet in your code or you can also continue adding your code in the same file
'''

from pyzbar import pyzbar
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy
from std_msgs.msg import String

class image_proc():

	# Initialise everything
	def __init__(self):
		rospy.init_node('qr_detect') #Initialise rosnode
		self.decode_data = rospy.Publisher('/qr_data', String, queue_size=1)
		self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
		self.img = np.empty([]) # This will contain your image frame from camera
		self.bridge = CvBridge()


	# Callback function of Camera topic
	def image_callback(self, data):
                
                try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
                        #print(self.img)
			cv2.imshow('image',self.img)
                        cv2.waitKey(3)
                        barcodes = pyzbar.decode(self.img)
                        for barcode in barcodes:
                                # extract the bounding box location of the barcode and draw the
                                # bounding box surrounding the barcode on the image
                                (x, y, w, h) = barcode.rect
                                cv2.rectangle(self.img, (x, y), (x + w, y + h), (0, 0, 255), 2)
                                # the barcode data is a bytes object so if we want to draw it on
                                # our output image we need to convert it to a string first
                                barcodeData = barcode.data.decode("utf-8")
                                barcodeType = barcode.type
                                # draw the barcode data and barcode type on the image
                                text = "{} ({})".format(barcodeData, barcodeType)
                                cv2.putText(self.img, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                                        0.5, (0, 0, 255), 2)
                                # print the barcode type and data to the terminal
                                #print("[INFO] Found {} barcode: {}".format(barcodeType, barcodeData))
                                if barcodeType == "QRCODE":
                                        self.decode_data.publish(barcodeData)
                                else:
                                        self.decode_data.publish("Data not found")
                                        
                                
		except CvBridgeError as e:
			print(e)
			return

if __name__ == '__main__':
    image_proc_obj = image_proc()
    rospy.spin()
