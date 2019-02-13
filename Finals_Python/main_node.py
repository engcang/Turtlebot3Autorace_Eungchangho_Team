#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri Nov  9 10:25:22 2018

@author: mason
"""
'''libraries'''
import time
import numpy as np
import subprocess
import rospy
import roslib
import cv2
import sys
import signal

from math import pow,atan2,sqrt,sin,cos
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
#from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import UInt8
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion, quaternion_from_euler
#from cv_bridge import CvBridge, CvBridgeError

global junction_passed
global left
global right
left=right=0
junction_passed=0

global LSD
global LSD2
global LSD3
global LSD4
LSD = cv2.createLineSegmentDetector(0)
LSD2 = cv2.createLineSegmentDetector(1)
LSD3 = cv2.createLineSegmentDetector(2)
LSD4 = cv2.createLineSegmentDetector(3)

global check_odom_x
global check_odom_y
check_odom_x=check_odom_y=0
global odom_ok
odom_ok = 0

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

''' class '''
class robot():
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pose_subscriber = rospy.Subscriber('/odom', Odometry, self.callback_pose)
        self.lidar_subscriber = rospy.Subscriber('/scan',LaserScan,self.callback_lidar)
        self.sign_subscriber = rospy.Subscriber('/sign_flag',UInt8,self.callback_sign)
        self.sonar_subscriber = rospy.Subscriber('/sonar_dist',Float32, self.callback_sonar)
        #self.img_publisher = rospy.Publisher('/mason_image',Image,queue_size=1)
        #self.img_publisher = rospy.Publisher('/after_image',CompressedImage,queue_size=1)
        self.img_subscriber = rospy.Subscriber('/raspicam_node/image/compressed',CompressedImage,self.callback_img)
        #self.bridge = CvBridge()
        self.f=0
        self.dist=0

    def callback_img(self,data):
        np_arr = np.fromstring(data.data, np.uint8)
        self.image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        
    def callback_sonar(self,data):
        self.dist=data.data
        
    def callback_sign(self,data):
        global junction_passed
        global left
        global right
        global check_odom_x
        global check_odom_y
        self.f=data.data
        if self.f==2:
            junction_passed=0.04
            left=1
            check_odom_x=self.pose.x
            check_odom_y=self.pose.y
        if self.f==3:
            junction_passed=0.04
            right=1
            check_odom_x=self.pose.x
            check_odom_y=self.pose.y
    def callback_pose(self, data):
        global odom_ok
        global check_odom_x
        global check_odom_y
        global left
        global right
        self.pose = data.pose.pose.position
        self.orient = data.pose.pose.orientation
        orientation_list = [self.orient.x, self.orient.y, self.orient.z, self.orient.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.theta = yaw
        if odom_ok==0 and (self.pose.x<check_odom_x-0.6 and self.pose.y>check_odom_y+0.2) :
            odom_ok=1
            left=0
            right=1
            
    def callback_lidar(self, data):
        self.scan = data.ranges
    def moving(self,vel_msg):
        self.velocity_publisher.publish(vel_msg)
    def stop(self):
        vel_msg=Twist()
        vel_msg.linear.x=0
        vel_msg.angular.z=0
        self.velocity_publisher.publish(vel_msg)
    def going(self,centimeter):
        vel_msg=Twist()
        if centimeter>0:
            vel_msg.linear.x=0.2
            vel_msg.angular.z=0
            self.velocity_publisher.publish(vel_msg)
        else:
            vel_msg.linear.x=-0.2
            vel_msg.angular.z=0
            self.velocity_publisher.publish(vel_msg)
        time.sleep(abs(centimeter/20.0)) ## fucking .0
        self.stop()
    def turn(self,deg):
        degree=deg/180.0*np.pi # fucking .0
        vel_msg=Twist()
        if degree <0 :
            vel_msg.angular.z=-1
            vel_msg.linear.x=0
            self.velocity_publisher.publish(vel_msg)
        elif degree>0:
            vel_msg.angular.z=1
            vel_msg.linear.x=0
            self.velocity_publisher.publish(vel_msg)
        time.sleep(abs(degree))
        self.stop()
    def move2goal(self, goal_x, goal_y):
        distance_tolerance = 0.05
        vel_msg = Twist()
        r = sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2))
        while r >= distance_tolerance:
            r = sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2))
            psi = atan2(goal_y - self.pose.y, goal_x - self.pose.x)
            phi = self.theta - psi
            if phi > np.pi:
                phi = phi - 2*np.pi
            if phi < -np.pi:
                phi = phi + 2*np.pi

            vel_msg.linear.x = 0.6*0.3*cos(phi)
            vel_msg.angular.z = -0.6*sin(phi)*cos(phi)-(0.6*phi)
            self.moving(vel_msg)
            self.rate.sleep()
        self.stop()
        
    def keeping(self,hsv):
        global junction_passed
        global LSD
        global left
        global right
        vel_msg=Twist()
	#crop=hsv[340:400,20:582]
        crop_L=hsv[350:410,40:220]
        crop_R=hsv[350:410,402:582]
        L_mask = cv2.inRange(crop_L,(24,50,100),(36,255,255))
        L2_mask = cv2.inRange(crop_L,(36,0,165),(255,255,255))
        R_mask = cv2.inRange(crop_R,(36,0,165),(255,255,255))
        R2_mask = cv2.inRange(crop_R,(24,50,100),(36,255,255))
      
        #index_L_mask = L_mask>0
        #index_R_mask = R_mask>0
        #index_L2_mask = L2_mask>0
        #index_R2_mask = R2_mask>0
    
        #yello=np.zeros_like(crop_L,np.uint8)
        #white=np.zeros_like(crop_R,np.uint8)
        #yello[index_L_mask]= crop_L[index_L_mask]
        #yello[index_L2_mask]= crop_L[index_L2_mask]
        #white[index_R_mask]=crop_R[index_R_mask]
        #white[index_R2_mask]=crop_R[index_R2_mask]
        yello_line = LSD.detect(L_mask)
        yello_line2 = LSD.detect(L2_mask)
        white_line = LSD.detect(R_mask)
        white_line2 = LSD.detect(R2_mask)
        if left==1:
            if yello_line[0] is None and yello_line2[0] is None:
                vel_msg.linear.x = 0.05
                vel_msg.angular.z = 0.6
            elif white_line[0] is None and white_line2[0] is None:
                vel_msg.linear.x = 0.05
                vel_msg.angular.z = -0.6
            else :
                vel_msg.linear.x = 0.16 + junction_passed
                vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)
            #cv2.imshow('hsv',yello)
            #cv2.imshow('hsv',white)
            #cv2.imshow('hsv',crop)
            #if cv2.waitKey(1)>0: return
        elif right==1:
            if white_line[0] is None and white_line2[0] is None:
                vel_msg.linear.x = 0.05
                vel_msg.angular.z = -0.6
            elif yello_line[0] is None and yello_line2[0] is None:
                vel_msg.linear.x = 0.05
                vel_msg.angular.z = 0.6
            else :
                vel_msg.linear.x = 0.16 + junction_passed
                vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)
            #cv2.imshow('hsv',yello)
            #cv2.imshow('hsv',white)
            #cv2.imshow('hsv',crop)
            #if cv2.waitKey(1)>0: return
        else:
            if yello_line[0] is None and yello_line2[0] is None:
                vel_msg.linear.x = 0.05
                vel_msg.angular.z = 0.6
            elif white_line[0] is None and white_line2[0] is None:
                vel_msg.linear.x = 0.05
                vel_msg.angular.z = -0.6
            else :
                vel_msg.linear.x = 0.16 + junction_passed
                vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)
            #cv2.imshow('hsv',yello)
            #cv2.imshow('hsv',white)
            #cv2.imshow('hsv',crop)
            #if cv2.waitKey(1)>0: return

    def keeping_parking(self,hsv):
        global LSD2
        vel_msg=Twist()
        crop_L=hsv[340:410,40:220]
        crop_R=hsv[340:410,402:582] # ranges are different
        L_mask = cv2.inRange(crop_L,(24,50,100),(36,255,255))
        L2_mask = cv2.inRange(crop_L,(36,0,165),(255,255,255))
        R_mask = cv2.inRange(crop_R,(36,0,165),(255,255,255))
        R2_mask = cv2.inRange(crop_R,(24,50,100),(36,255,255))
      
        yello_line = LSD2.detect(L_mask)
        yello_line2 = LSD2.detect(L2_mask)
        white_line = LSD2.detect(R_mask)
        white_line2 = LSD2.detect(R2_mask)
        if yello_line[0] is None and yello_line2[0] is None:
            vel_msg.linear.x = 0.05
            vel_msg.angular.z = 0.5
        elif white_line[0] is None and white_line2[0] is None:
            vel_msg.linear.x = 0.05
            vel_msg.angular.z = -0.5
        else :
            vel_msg.linear.x = 0.16
            vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
            
    def poseupdate(self):
        return self.pose.x, self.pose.y, self.theta
    def scanupdate(self):
        return self.scan

    def imageupdate(self):
        #global cap
        #ret, image=cap.read()
        image=self.image_np
        hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        return image,hsv
    #def img_sendor(self,img):
    #    self.img_publisher.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
    #def img_sendor(self,img):
    #    image=CompressedImage()
    #    image.data=np.array2string(img)
    #    self.img_publisher.publish(image)
        
    def wall_avoiding(self):
        global left
        global right
        right=0
        left = 1
        self.stop()
        p=subprocess.Popen('rostopic pub /reset std_msgs/Empty "{}"',shell=True)
        time.sleep(1.2)
        p.terminate()
        time.sleep(1.3)
        '''self.move2goal(0.5116, 0.05)
        self.move2goal(0.5831, 0.08)
        self.move2goal(0.6387, 0.3057)
        self.move2goal(0.4955, 0.6119)
        self.move2goal(0.4973, 0.9031)
        self.move2goal(0.6485, 1.1533)
        self.move2goal(0.5854, 1.3716)
        self.move2goal(0.4380, 1.4383)
        self.move2goal(0.1678, 1.4854)
        self.move2goal(0.1297, 1.4853)'''
        self.move2goal(0.5116, 0.05)
        self.move2goal(0.5831, 0.08)
        self.move2goal(0.6387, 0.3057)
        self.move2goal(0.4955, 0.6119)
        self.move2goal(0.4973, 0.8831)
        self.move2goal(0.5473, 0.9031)
        self.move2goal(0.6485, 1.1533)
        self.move2goal(0.5854, 1.3716)
        self.move2goal(0.4380, 1.4383)
        self.move2goal(0.1678, 1.4854)
        self.move2goal(0.1497, 1.4853)
        self.stop()
        
    def parking2(self):
        global left
        global right
        right= 0
        left = 1
        self.stop()
        time.sleep(0.5)
        p=subprocess.Popen('rostopic pub /reset std_msgs/Empty "{}"',shell=True)
        time.sleep(1.2)
        p.terminate()
        time.sleep(1.3)
        while self.pose.x<0.2:
            img,hsv = self.imageupdate()
            self.keeping_parking(hsv)
        while self.pose.y<0.975:
            img,hsv = self.imageupdate()
            self.keeping_parking(hsv)
        self.stop()
        judge=10*np.ones([360,1])
        for i in range(210,331):
            if self.scan[i]!=0:
                judge[i]=self.scan[i]
        minx=np.amin(judge)
        if minx>0.4 : #right
            self.turn(-91)
            self.going(34)
            time.sleep(1.5)
            self.going(-34)
            self.turn(-91)
        else: #left
            self.turn(91)
            self.going(34)
            time.sleep(1.5)
            self.going(-34)
            self.turn(91)
        self.stop()
        time.sleep(0.5)
        while self.pose.y>0.08:
            img,hsv = self.imageupdate()
            self.keeping(hsv)
        self.turn(30)

    def keeping_tunnel_enter(self):
        global LSD4
        img,hsv = self.imageupdate()
        vel_msg=Twist()
        crop_L=hsv[345:410,40:220]
        crop_R=hsv[345:410,402:582] # ranges are different
        L_mask = cv2.inRange(crop_L,(24,50,100),(36,255,255))
        L2_mask = cv2.inRange(crop_L,(36,0,165),(255,255,255))
        R_mask = cv2.inRange(crop_R,(36,0,165),(255,255,255))
        R2_mask = cv2.inRange(crop_R,(24,50,100),(36,255,255))
      
        yello_line = LSD4.detect(L_mask)
        yello_line2 = LSD4.detect(L2_mask)
        white_line = LSD4.detect(R_mask)
        white_line2 = LSD4.detect(R2_mask)
        if (yello_line[0] is not None or yello_line2[0] is not None) and (white_line[0] is not None or white_line2[0] is not None): #dul da
            vel_msg.linear.x = 0.05
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)
            return 1
        elif (yello_line[0] is None and yello_line2[0] is None) and (white_line[0] is None and white_line2[0] is None): # nothing
            self.stop()
            return 2
        elif yello_line[0] is None and yello_line2[0] is None:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0.3
            self.velocity_publisher.publish(vel_msg)
            return 1
        elif white_line[0] is None and white_line2[0] is None:
            vel_msg.linear.x = 0
            vel_msg.angular.z = -0.3
            self.velocity_publisher.publish(vel_msg)
            return 1

    def keeping_tunnel(self):
        global LSD3
        img,hsv = self.imageupdate()
        crop_L=hsv[355:410,40:220]
        crop_R=hsv[355:410,402:582] # ranges are different
        L_mask = cv2.inRange(crop_L,(24,50,100),(36,255,255))
        L2_mask = cv2.inRange(crop_L,(36,0,165),(255,255,255))
        R_mask = cv2.inRange(crop_R,(36,0,165),(255,255,255))
        R2_mask = cv2.inRange(crop_R,(24,50,100),(36,255,255))
      
        yello_line = LSD3.detect(L_mask)
        yello_line2 = LSD3.detect(L2_mask)
        white_line = LSD3.detect(R_mask)
        white_line2 = LSD3.detect(R2_mask)
        if yello_line[0] is not None or yello_line2[0] is not None:
            return 1
        elif white_line[0] is not None or white_line2[0] is not None:
            return 1
        else :
            return 2

    def tunnel(self):
        vel_msg=Twist()
        while self.keeping_tunnel_enter()==1:
            pass
        print('tunnel entered')
        self.going(20)

        temp_center_1 = np.zeros((20,1))
        median_center_1 = np.zeros((16,1))
        temp_center_2 = np.zeros((20,1))
        median_center_2 = np.zeros((16,1))
        temp_center_3 = np.zeros((20,1))
        median_center_3 = np.zeros((16,1))
        temp_center_4 = np.zeros((20,1))
        median_center_4 = np.zeros((16,1))
        temp_center_5 = np.zeros((20,1))
        median_center_5 = np.zeros((16,1))

        temp_right_1 = np.zeros((20,1))
        median_right_1 = np.zeros((16,1))
        temp_right_2 = np.zeros((20,1))
        median_right_2 = np.zeros((16,1))

        data = np.zeros((360,1))
        th = 0.2

        while self.keeping_tunnel()==2:
            sum_c_1 = 0
            num_c_1 = 0
            sum_c_2 = 0
            num_c_2 = 0
            sum_c_3 = 0
            num_c_3 = 0
            sum_c_4 = 0
            num_c_4 = 0
            sum_c_5 = 0
            num_c_5 = 0

            sum_r_1 = 0
            sum_r_2 = 0
            num_r_1 = 0
            num_r_2 = 0

            data[:,0] = self.scanupdate()
            for j in range(0, 360) :
                if (data[j] < 0.05 or data[j] ==0):
                    data[j] = 3

            # center	
            temp_center_1[0:19] = data[310:329]
            temp_center_2[0:19] = data[330:349]
            temp_center_3[0:9] = data[350:359]
            temp_center_3[10:19] = data[0:9]
            temp_center_4[0:19] = data[10:29]
            temp_center_5[0:19] = data[30:49]

            # right
            temp_right_1[0:19] = data[270:289]
            temp_right_2[0:19] = data[250:269]

            for j in range(2, 16):
                median_center_1[j] = np.median(temp_center_1[j-2:j+2])
                median_center_2[j] = np.median(temp_center_2[j-2:j+2])
                median_center_3[j] = np.median(temp_center_3[j-2:j+2])
                median_center_4[j] = np.median(temp_center_4[j-2:j+2])
                median_center_5[j] = np.median(temp_center_5[j-2:j+2])

                median_right_1[j] = np.median(temp_right_1[j-2:j+2])
                median_right_2[j] = np.median(temp_right_2[j-2:j+2])
		
                if median_center_1[j] > 0.1 :
                    sum_c_1 = sum_c_1 + median_center_1[j]
                    num_c_1 = num_c_1 + 1
    
                if median_center_2[j] > 0.1 :
                    sum_c_2 = sum_c_2 + median_center_2[j]
                    num_c_2 = num_c_2 +1
    
                if median_center_3[j] > 0.1 :
                    sum_c_3 = sum_c_3 + median_center_3[j]
                    num_c_3 = num_c_3 + 1
    
                if median_center_4[j] > 0.1 :
                    sum_c_4 = sum_c_4 + median_center_4[j]
                    num_c_4 = num_c_4 +1
    
                if median_center_5[j] > 0.1 :
                    sum_c_5 = sum_c_5 + median_center_5[j]
                    num_c_5 = num_c_5 +1   
    
                if median_right_1[j] > 0.1 :
                    sum_r_1 = sum_r_1 + median_right_1[j]
                    num_r_1 = num_r_1 + 1
    
                if median_right_2[j] > 0.1 :
                    sum_r_2 = sum_r_2 + median_right_2[j]
                    num_r_2 = num_r_2 + 1

            if num_c_1 ==0:
                sum_c_1 = 10.0
            else :
                sum_c_1 = sum_c_1 / num_c_1
		     
            if num_c_2 ==0:
                sum_c_2 = 10.0
            else :
                sum_c_2 = sum_c_2 / num_c_2
		     
            if num_c_3 == 0:
                sum_c_3 = 10.0
            else :
                sum_c_3 = sum_c_3 / num_c_3
		     
            if num_c_4 ==0:
                sum_c_4 = 10.0
            else :
                sum_c_4 = sum_c_4 / num_c_4

            if num_c_5 ==0:
                sum_c_5 = 10.0
            else :
                sum_c_5 = sum_c_5 / num_c_5             

            if num_r_1 == 0:
                sum_r_1 = 10.0
            else :
                sum_r_1 = sum_r_1 / num_r_1

            if num_r_2 == 0:
                sum_r_2 = 10.0
            else :
                sum_r_2 = sum_r_2 / num_r_2

            print([sum_c_1, sum_c_2, sum_c_3, sum_c_4, sum_c_5, sum_r_1, sum_r_2])
            if ( sum_c_2 > 1 and sum_c_3 > 1 and sum_c_4 >1) : 	
                vr = 0.15
                wr = -0.1
            elif (sum_c_1 < 0.3 or sum_c_2 < 0.28 or sum_c_3 < 0.27 or sum_c_4 < 0.28  or sum_c_5 < 0.3) : 
                if (sum_r_1 < 0.3 or sum_r_1 < 0.3) :
                    vr = 0.03
                    wr = 1
                else :
                    vr = 0.03
                    wr = -1   
            else:
                vr = 0.15
                wr = 0

            vel_msg.linear.x = vr
            vel_msg.angular.z = wr
            self.moving(vel_msg)
        print('talchoolllllllllllllllllllllll')
        self.stop()
        
''' functions '''
def tic():
    global starttime
    starttime=time.time()

def toc():
    nowtime=time.time()
    print("toc: %f"%(nowtime-starttime))
#    return nowtime-starttime

turtle=robot()
time.sleep(1.2)
if __name__=='__main__':
    '''turtle.f=0
    while turtle.f==0:
        try:
            img,hsv = turtle.imageupdate()
            #turtle.img_sendor(img)
        except:
            print('errrr')'''
    #turtle.f=7
    while 1:
        tic()
        try:
            img,hsv=turtle.imageupdate()
            #turtle.img_sendor(img)
            #print(turtle.f)
            if turtle.f==4:   # wall avoiding
                turtle.wall_avoiding()
            elif turtle.f== 5:   # parking
                turtle.parking2()
            elif turtle.f==8: # slow, ready for stop
                junction_passed=-0.11
            elif turtle.f== 6:   # stop 
                turtle.stop()
                time.sleep(7)
                junction_passed=0.04
                left=0
                right=1
            elif turtle.f== 7:   # Tunnel
                turtle.tunnel()
                turtle.f=0
            if turtle.f!=6:
                turtle.keeping(hsv) 
        except (KeyboardInterrupt, SystemExit):
            sys.exit(0)
        except :
           print('got error')
        toc()
