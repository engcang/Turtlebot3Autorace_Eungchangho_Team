#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat Nov 10 05:05:53 2018

@author: mason
"""
import rospy
import cv2
import numpy as np
import time
import sys
import signal

#from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import UInt8
#from cv_bridge import CvBridge, CvBridgeError

global traffic_passed 
global stop_sign_detected
global junction_passed 
global wall_passed 
global parking_passed
global left 
global right
left=0; right=0; wall_passed=0; parking_passed=0;
junction_passed=0; traffic_passed=1; stop_sign_detected=0;
global incoming
incoming=0
global itera
itera=0

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

class sign_detector():
    def __init__(self):
        rospy.init_node('sign_detector', anonymous=True)
        self.sign_publisher = rospy.Publisher('/sign_flag',UInt8,queue_size=1)
        #self.img_subscriber = rospy.Subscriber('/mason_image',Image,self.callback_img)
        #self.img_subscriber = rospy.Subscriber('/after_image',CompressedImage,self.callback_img)
        self.img_subscriber = rospy.Subscriber('/raspicam_node/image/compressed',CompressedImage,self.callback_img)
        self.rate = rospy.Rate(20)
        #self.bridge = CvBridge()
        self.f=0
        
        
    def callback_img(self,data):
        #global itera
        #itera = itera +1
        #if itera == 5:
        global incoming
        incoming =1
        #self.cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        np_arr = np.fromstring(data.data, np.uint8)
        self.cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        #itera=0
        
    def img_update(self):
        img=self.cv_img
        hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        return img,hsv
    
    def sign_sendor(self):
        data = UInt8()
        data.data=self.f
        self.sign_publisher.publish(data)

def check_sign(img,hsv):
    global traffic_passed 
    global stop_sign_detected
    global junction_passed 
    global wall_passed 
    global parking_passed
    global left 
    global right
    f=0
    scale=1.2 #resize .*100%, smaller detect more but take longer
    minNeighbors=5 # higer, higer quality, less detection
    if traffic_passed == 0:
        if color_pixel_number(hsv,[(55,100,50),(75,255,180)],[[320,100,180,120]]) > 200:
            f = 1
            traffic_passed = 1
            print('traffic_detected')
    else:
        if junction_passed==0:
            detector_turn_left = cv2.CascadeClassifier('left.xml')
            detector_turn_right = cv2.CascadeClassifier('right.xml')
            bbox_turn_left=detector_turn_left.detectMultiScale(img,scale,minNeighbors)
            bbox_turn_right=detector_turn_right.detectMultiScale(img,scale,minNeighbors)
            if np.shape(bbox_turn_left)[0]!=0:
                if color_pixel_number(hsv,[(105,150,70),(110,255,255)],bbox_turn_left)>400:
                    f=2
                    junction_passed=0.04
                    left=1
                    print('junction_detected')
            if np.shape(bbox_turn_right)[0] !=0:
                if color_pixel_number(hsv,[(105,150,70),(110,255,255)],bbox_turn_right)>400:
                    f=3
                    junction_passed=0.04
                    right=1
                    print('junction_detected')
        elif junction_passed!=0 and wall_passed==0:
            detector_wall = cv2.CascadeClassifier('wall.xml') #wall2.xml
            bbox_wall = detector_wall.detectMultiScale(img,scale,minNeighbors)
            if np.shape(bbox_wall)[0] !=0:
                if color_pixel_number(hsv,[(11,150,50),(20,255,255)],bbox_wall)>380:
                    f=4
                    wall_passed=1
                    left=1
                    print('wall_detected')
                    #cv2.imshow('fucking',cv2.inRange(hsv,(14,150,50),(20,255,255)))
                    #cv2.waitKey()
        elif wall_passed==1 and parking_passed==0:
            if color_pixel_number(hsv,[(100,110,30),(108,255,120)],[[390,50,230,150]])>1000: #[(103,110,30),(117,255,120)],[[320,100,200,150]]
                f=5
                parking_passed=1
                left=1
                print('parking_detected')
            '''detector_parking = cv2.CascadeClassifier('parking.xml')
            bbox_parking = detector_parking.detectMultiScale(img,scale,minNeighbors)
            if np.shape(bbox_parking)[0] !=0:
                if color_pixel_number(hssv,[(103,80,30),(110,255,120)],bbox_parking)>500:
                    f=5
                    parking_passed=1
                    left=1
                    print('parking_detected')'''
        elif parking_passed==1 and stop_sign_detected==0:
            #detector_stop = cv2.CascadeClassifier('stop.xml')
            #bbox_stop = detector_stop.detectMultiScale(img,scale,minNeighbors)
            #if np.shape(bbox_stop)[0] != 0 and stop_sign_detected==0:40:200,50:630
            if color_pixel_number(hsv,[(170,70,50),(180,255,120)],[[50,40,580,160]])>5350:
                stop_sign_detected=1
                print('stop BARRRRR')
                f=6
            elif color_pixel_number(hsv,[(170,70,50),(180,255,120)],[[50,40,580,160]])>700:
                f=8
                print('stop sign detected')
        elif stop_sign_detected==1:
            detector_tunnel = cv2.CascadeClassifier('tunnel.xml') #tunnel.xml
            bbox_tunnel = detector_tunnel.detectMultiScale(img,scale,minNeighbors)
            if np.shape(bbox_tunnel)[0] != 0:
                if color_pixel_number(hsv,[(14,130,50),(24,255,190)],bbox_tunnel) >= 350:
                    f = 7
                    print('tunnel detected, good luck')
            #if  color_pixel_number(hsv,[(170,60,50),(180,255,120)],[[130,40,370,100]]) >= 1100:
            #    f = 6
            #    print('bar detecting, stop')
    return f

def color_pixel_number(hsv,color,bbox):
    if np.shape(bbox)[0]>1:
        area=np.zeros(np.shape(bbox)[0])
        for i in range(np.shape(bbox)[0]):
            area[i]=bbox[i][2]*bbox[i][3]
        I=list(area).index(max(area))
        mask = cv2.inRange(hsv[bbox[I][1]:bbox[I][1]+bbox[I][3],bbox[I][0]:bbox[I][0]+bbox[I][2]],color[0],color[1])
        index_mask=mask>0
    else:
        mask = cv2.inRange(hsv[bbox[0][1]:bbox[0][1]+bbox[0][3],bbox[0][0]:bbox[0][0]+bbox[0][2]],color[0],color[1])
        index_mask=mask>0
    print (np.sum(index_mask))
    return np.sum(index_mask)
        
sign=sign_detector()
incoming=0
time.sleep(1)

while 1:
    if incoming==1:
        try:
            img,hsv=sign.img_update()
            sign.f=check_sign(img,hsv)
            sign.sign_sendor()
            incoming=0
        except (KeyboardInterrupt, SystemExit):
            sys.exit(0)
        except:
            print('img not yet')
    else:
        pass
