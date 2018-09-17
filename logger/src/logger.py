#!/usr/bin/env python
import rospy
from std_msgs.msg import String


def callback(req):
    print(str(req.data))

def listener():
    rospy.init_node('logger',anonymous = False)
    rospy.Subscriber("ardu_log_pub", String, callback)
    rospy.Subscriber("par_log_pub", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
