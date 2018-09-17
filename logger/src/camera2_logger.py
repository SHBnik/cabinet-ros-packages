#!/usr/bin/env python

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String

from sensor_msgs.msg import CameraInfo

# OpenCV2 for saving an image
import cv2
import time
import os
from os.path import expanduser
import datetime
from sounds import *


dir = ''

# Instantiate CvBridge
bridge = CvBridge()



action = ''
first_on = True

def set_action(msg):
    global action,current_frame
    try:
        rospy.loginfo(msg)
        action = msg.data
        action = action.split(',')
        with open(dir + '/camera2_logger.csv', 'a') as log_file :
            if action[1] == 'None':
                log_file.write('%d,%d,%s\n'%(current_frame,2,action[0]))
                action = action[0]
            else:
                log_file.write('%d,%d,%s\n'%(current_frame,2,sounds[int(action[1])]))
                action = sounds[int(action[1])]


    except CvBridgeError, e:
        print(e)





writer_flag = False
data_writer = None

def image_callback(msg):
    global data_writer,writer_flag

    #print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")

        cv2_img = cv2.flip(cv2_img,1)


        if end_process == False:

            if writer_flag == True :
                fourcc = cv2.VideoWriter_fourcc(*'MJPG')
                address = dir + '/camera2.avi'
                rospy.loginfo(address)
                data_writer = cv2.VideoWriter(address,fourcc, 30.0, (640,480))
                writer_flag = False


            font = cv2.FONT_HERSHEY_SIMPLEX
            try:
                cv2.putText(cv2_img,action,(1,25), font, 1,(0,0,255),2,cv2.LINE_AA)
                # cv2.putText(cv2_img,str(current_frame),(50,50), cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,255),2,cv2.LINE_AA)
            except Exception as e:
                print e


            try:
                data_writer.write(cv2_img)
            except Exception as e:
                print e

        elif end_process == True :
            try :
                data_writer.release()
            except Exception as e :
                print e


        # Save your OpenCV2 image as a jpeg
        cv2.imshow('camera_image2', cv2_img)
        key = cv2.waitKey(1)
    except CvBridgeError, e:
        print(e)



first_seq = -1
current_frame = 0

def seq_callback(data):
    global first_seq,current_frame
    if end_process == False:
        if first_seq == -1:
            first_seq = data.header.seq
        else:
            current_frame = data.header.seq - first_seq
    else:
        first_seq = -1
        current_frame = 0





uid = None
end_process = True

def start_end_logging(data):
    global end_process,uid,action
    if data.data == '-1':
        end_process = True
        action = None
    else :
        uid = data.data
        end_process = False



def set_dir(data):
    global dir,writer_flag
    dir = str(data.data)
    print dir
    writer_flag = True



def main():
    global data_writer

    rospy.init_node('camera2_listener')
    # Define your image topic
    image_topic = "/camera2/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)

    rospy.Subscriber('web/parrot_commands', String, set_action)

    rospy.Subscriber("/camera2/camera_info", CameraInfo, seq_callback)


    rospy.Subscriber("web/patient_uid", String, start_end_logging)

    rospy.Subscriber('web/patient_uid/dir', String, set_dir)


    # Spin until ctrl + c
    rospy.spin()
    try:
        data_writer.release()
    except: pass

if __name__ == '__main__':
    main()
