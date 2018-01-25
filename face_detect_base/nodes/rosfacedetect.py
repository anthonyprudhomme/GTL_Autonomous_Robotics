#!/usr/bin/python
"""
This program is demonstration for face and object detection using haar-like features.
The program finds faces in a camera image or video stream and displays a red box around them.

Original C implementation by:  ?
Python implementation by: Roman Stanchak, James Bowman
"""
import roslib
roslib.load_manifest('face_detect_base')

import sys
import os

import rospy
import sensor_msgs.msg
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import *
import numpy as np
from face_detect_base.msg import ROIArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


min_size = (10, 10)
image_scale = 2
haar_scale = 1.2
min_neighbors = 2
haar_flags = 0
display = True


if __name__ == '__main__':
    pub = rospy.Publisher('rois', ROIArray, queue_size=10)
    pub_img = rospy.Publisher('face', Image, queue_size=10)
    pub_marker = rospy.Publisher('face_markers', MarkerArray)
    opencv_dir = '/usr/share/opencv/haarcascades/';

    face_cascade = cv2.CascadeClassifier(opencv_dir + 'haarcascade_frontalface_default.xml')
    if face_cascade.empty():
        print "Could not find face cascade"
        sys.exit(-1)
    eye_cascade = cv2.CascadeClassifier(opencv_dir + 'haarcascade_eye.xml')
    if eye_cascade.empty():
        print "Could not find eye cascade"
        sys.exit(-1)
    br = CvBridge()
    rospy.init_node('facedetect')
    display = rospy.get_param("~display",True)

    def detect_and_draw(imgmsg):
        bridge = CvBridge()
        ROIs=ROIArray()
        listOfFaces=[]
        markerArray = MarkerArray()
        img = br.imgmsg_to_cv2(imgmsg, "bgr8")
        # allocate temporary images
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.3, 3)
        for (x,y,w,h) in faces:
            cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
            roi_gray = gray[y:y+h, x:x+w]
            roi_color = img[y:y+h, x:x+w]
            eyes = eye_cascade.detectMultiScale(roi_gray)
	    roi_face=[x,y,h,w,False]
            ROI=RegionOfInterest(x,y,h,w,False)
            ROIs.ROIArray.append(ROI)
            listOfFaces+=[roi_face]
            marker = Marker()
            marker.ns = str(len(listOfFaces))
            marker.id = len(listOfFaces)
            marker.header.frame_id = "/body"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = y/10
            marker.pose.position.y = h/20 
            marker.pose.position.z = 0.4
            markerArray.markers.append(marker)
            for (ex,ey,ew,eh) in eyes:
                cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)
        listOfFaces=np.array(listOfFaces)
        #print(listOfFaces)
        pub.publish(ROIs)
        cv2.imshow('img',img)
        cv2.waitKey(10)
        pub_img.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
        pub_marker.publish(markerArray)
	
    rospy.Subscriber("~image", sensor_msgs.msg.Image, detect_and_draw)
    rospy.spin()
