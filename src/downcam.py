#!/usr/bin/env python
import cv2
import numpy as np
import rospkg
import rospy
from sensor_msgs.msg import CompressedImage

lower_orange = np.array([0,0,233])
upper_orange = np.array([150,12,255])
lower_red = np.array([0,0,0])
upper_red = np.array([40,200,255])
img = None 
contours = None
orange = None

def  delNoise():
    global orange
    mask = orange
    erode = cv2.erode(mask,np.array([[1]*3]*3))
    dilate = cv2.dilate(erode, np.array([[1]*5]*5))
    _,contours,hierarchy = cv2.findContours(dilate.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

    result = dilate.copy()
    result = cv2.cvtColor(result,cv2.COLOR_GRAY2BGR)
    sq_found = []

    for c in contours:
		M = cv2.moments(c)
		# y x 
		rect = (x,y),(ww,hh),angle = cv2.minAreaRect(c)
		area = ww*hh
		
		if area <= 0 or ww <= 0 :
			continue
		real_area = cv2.contourArea(c)
		ratio_area = real_area/area
		ratio_scale = hh/ww

		box = cv2.boxPoints(rect)
		box = np.int0(box)
	#	print real_area, ratio_area, ratio_scale 
		err = 3
		
		if  ratio_area > 0.6 and real_area >= 200 and (ratio_scale > 3 or ratio_scale < 0.33) and (y-hh/2 > err and y+hh/2 < h-err and x-ww/2 > err and x+ww/2 < w-err or real_area >= 200):
	#		print '->>>',real_area, ratio_area, ratio_scale 
			sq_found.append([real_area,(x,y),90-Oreintation(M)[0]*180/math.pi])
			cv2.drawContours(result,[box],0,(0,255,0),1,8)
	
    if len(sq_found) == 0:
	    not_found(m)
    else:
		sq_found = sorted(sq_found,key = itemgetter(0))
		x,y = sq_found[-1][1]
		cv2.circle(result,(int(x),int(y)), 5,(0,0,255), -1)
	
		m.x = ((height/2.0) - y)/(height/2.0)
		m.y = ((width/2.0) - x)/(width/2.0)	
		lx, ly = m.x, m.y
		m.angle = sq_found[-1][2] 
		m.area = sq_found[-1][0] / (width*height)
		m.appear = True
    cv2.imshow('result',result)
    k = cv2.waitKey(1) & 0xff
    if k == ord('q'):
        rospy.signal_shutdown('')
    return m

def findContours():
    global contours,img,orange
    kernel = np.ones((5,5),np.uint8)
    while not rospy.is_shutdown():
        while img is None:
            print("img None")
        im = img.copy()
        blur = cv2.blur(im, (5,5))
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        orange = cv2.inRange(hsv, lower_orange, upper_orange)
        erosion = cv2.erode(orange, kernel, iterations = 1)
        dilation = cv2.dilate(orange, kernel, iterations = 1)
        closing = cv2.morphologyEx(orange, cv2.MORPH_CLOSE, kernel)
        opening = cv2.morphologyEx(orange, cv2.MORPH_OPEN, kernel)

        # cv2.imshow('img',orange )
        imgray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(orange,127,255,0)
        _ , contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,
                                                        cv2.CHAIN_APPROX_SIMPLE)
        result = cv2.drawContours(im, contours, -1, (0,255,0), 3)
        # cv2.imshow('img',result)
        cv2.imshow("blur", blur)
        cv2.imshow("erosion",erosion)
        cv2.imshow("dilation",dilation)
        cv2.imshow("closing",closing)
        cv2.imshow("opening", opening)
        cv2.waitKey(30)

def callback(msg):
    global img,wait,hsv,orange

    arr = np.fromstring(msg.data, np.uint8)
    img = cv2.imdecode(arr, 1)
    
def findPath():
    global img
    findContours()

if __name__ == '__main__':
    rospy.init_node('playback')
    topic = '/rightcam_bottom/image_raw/compressed'
    rospy.Subscriber(topic, CompressedImage,callback) 
    findPath()
    # delNoise()
            