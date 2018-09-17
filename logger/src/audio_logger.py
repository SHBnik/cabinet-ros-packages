#!/usr/bin/env python
import rospy
import os
from std_msgs.msg import String
from os.path import expanduser
import datetime
import subprocess
import time

def record_callback(req):
    if req.data == '-1':
        p.terminate()
        time.sleep(2)
    else:
        UID = req.data
        p = subprocess.Popen(['roslaunch','audio_capture','capture_to_file.launch','device:=1','dst:=%s/audio_log.mp3'%dir])

dir = ''
def set_dir(data):
    global dir,writer_flag
    dir = str(data.data)


def audio_logger():
    rospy.init_node('audio_logger',anonymous = False)
    rospy.Subscriber("/web/patient_uid", String, record_callback)
    rospy.Subscriber("/web/patient_uid/dir", String, set_dir)
    rospy.spin()


if __name__ == '__main__':
    audio_logger()
