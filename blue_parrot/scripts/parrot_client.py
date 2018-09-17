#!/usr/bin/env python
import rospy
from parrot.srv import *


def send_command(command, duration):
    rospy.wait_for_service('parrot')
    try:
        parrot = rospy.ServiceProxy('parrot', Parrot)
        resp1 = parrot(command, duration)
        return resp1.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def usage():
    return "%s [x y]"%sys.argv[0]


if __name__ == "__main__":
    if len(sys.argv) == 3:
        command = sys.argv[1]
        duration = int(sys.argv[2])
    else:
        print usage()
        sys.exit(1)
    print "Requesting command: %s with duration: %s"%(command, duration)
    print "command %s with duration %s -> %s"%(command, duration, send_command(command, duration))
