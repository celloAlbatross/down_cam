#!/usr/bin/env python
import cv2
import numpy as np
import rospkg
import rospy
from sensor_msgs.msg import CompressedImage

lower_orange = np.array([30,30,152])
upper_orange = np.array([150,100,255])
lower_red = np.array([0,0,0])
upper_red = np.array([40,200,255])
img = None 
contours = None
orange = None
closing = None

def find_path():
    global contours,img

    for c in contours:
        rect = (x,y),(ww,hh),angle = cv2.minAreaRect(c)
        area = ww*hh
        # print("area: ")
        # print(ww)
        if area <= 20 or area >= 1000 or ww <= 10:
            continue 
        real_area = cv2.contourArea(c)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(img, [box], -1,(0,0,0),1,8)

    # cv2.imshow('img' ,img)

def findColors():
    global contours,img,orange, closing
    kernel = np.ones((5,5),np.uint8)
    while not rospy.is_shutdown():
        while img is None:
            print("img None")
        im = img.copy()
        blur = cv2.blur(im, (5,5))
        blur2 = cv2.blur(blur, (5,5))
        hsv = cv2.cvtColor(blur2, cv2.COLOR_BGR2HSV)
        orange = cv2.inRange(hsv, lower_orange, upper_orange)
        opening = cv2.morphologyEx(orange, cv2.MORPH_OPEN, kernel)
        erosion = cv2.erode(orange, kernel, iterations = 1)
        dilation = cv2.dilate( orange, kernel, iterations = 1)
        closing = cv2.morphologyEx(orange, cv2.MORPH_CLOSE, kernel)

        # cv2.imshow('img',orange )
        imgray = cv2.cvtColor(blur,cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(closing,127,255,0)
        _ , contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,
                                                        cv2.CHAIN_APPROX_NONE)
        result = cv2.drawContours(im, contours, -1, (0,255,255), 3)
        find_path()
        # cnt = contours[0]
        # rect = cv2.minAreaReact(cnt)
        # box = cv2.boxPoint(rect)
        # box = np.int0(box)
        # res1 = cv2.drawContours(img, [box],0,(0,0,255) ,2)
        
        # print(contours)
        # cv2.imshow('orange', orange)
        cv2.imshow('img',result)
        # cv2.imshow("blur", blur2)
        # cv2.imshow("erosion",erosion)
        # cv2.imshow('hsv',hsv)
    	# cv2.imshow("dilation",dilation)
        # cv2.imshow("closing",closing)
        # cv2.imshow("opening", opening)
     	cv2.waitKey(30)

def callback(msg):
    global img,wait,hsv,orange

    arr = np.fromstring(msg.data, np.uint8)
    img = cv2.imdecode(arr, 1)


if __name__ == '__main__':
    rospy.init_node('playback')
    topic = '/rightcam_bottom/image_raw/compressed'
    rospy.Subscriber(topic, CompressedImage,callback) 
    findColors()
    # find_path()
    # delNoise()
            
