#!/usr/bin/env python
import cv2
import numpy as np
import rospkg
import rospy
from sensor_msgs.msg import CompressedImage

lower_orange = np.array([150,0,0])
upper_orange = np.array([80,200,255])
lower_red = np.array([0,0,0])
upper_red = np.array([40,200,255])
img = None 
contours = None

def findContours():
    global contours,img
    # print (img)
    while not rospy.is_shutdown():
        while img is None:
            print("img None")
        im = img.copy()
        hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        pink = cv2.inRange(hsv, lower_orange, upper_orange)
        red = cv2.inRange(hsv, lower_red, upper_red)
        orange = pink+red
        cv2.imshow('img',orange   )
        # cv2.waitKey(30)
        imgray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(orange,127,255,0)
        _ , contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,
                                                        cv2.CHAIN_APPROX_SIMPLE)
        result = cv2.drawContours(im, contours, -1, (0,255,0), 3)
        # cv2.imshow('img',result)
        # cv2.imshow('img',hsv)
        cv2.waitKey(30)

def callback(msg):
    global img,wait,hsv,orange

    arr = np.fromstring(msg.data, np.uint8)
    img = cv2.imdecode(arr, 1)
    # hsv = cv2.cvtColor(img, cv2.COLOR_BRG2HSV)
    # orange = cv2.inRange(hsv, lower_orange,upper_orange)

    # cv2.imshow('path',img)

    # cv2.waitKey(1) #& 0xFF == ord('q'):
    
def findPath():
    global img
    findContours()
    # cv2.imshow('img',img)
    # cv2.waitKey(30)
    

if __name__ == '__main__':
    rospy.init_node('playback')
    topic = '/rightcam_bottom/image_raw/compressed'
    rospy.Subscriber(topic, CompressedImage,callback) 
    findPath()
            