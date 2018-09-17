#!/usr/bin/env python
import rospy
import serial
import threading
import inspect
import time
from parrot.srv import *
from serial import SerialException
from sound_play.libsoundplay import SoundClient


DEFAULT_PORT = '/dev/ttyACM0'
P_PORT = 'port'
TEN_SECONDS = 10
CONNECTED = 'connected'
DISCONNECTED = 'disconnected'

CURTAIN_CLOSED = 'connected'
CURTAIN_OPENED = 'disconnected'

ser = serial.Serial()
lock = threading.Lock()
soundClient = SoundClient()
state = DISCONNECTED

curtain_state = CURTAIN_CLOSED


def send_serial_command(command):
    if lock.acquire(False):
        rospy.loginfo('slow down on sending serial commands')
        return

    try:
        ser.write(command)
        time.sleep(.5)
    except SerialException:
        rospy.loginfo('SerialException on write {}'.format(SerialException))
    finally:
        lock.release()


def start():
    global ser, state

    if state == CONNECTED:
        return

    port_param = '{}/{}'.format(rospy.get_name(), P_PORT)
    try:
        port = rospy.get_param(port_param)
    except KeyError:
        port = DEFAULT_PORT
        rospy.set_param(port_param, DEFAULT_PORT)

    rospy.loginfo('port is set to {}'.format(port))

    ser = serial.Serial()
    ser.port = port
    ser.baudrate = 9600
    ser.parity = serial.PARITY_NONE

    try:
        ser.open()
        state = CONNECTED
        while int(ser.readline()) != 85: continue
    except SerialException:
        state = DISCONNECTED
        soundClient.say('I can\'t connect to the port. Make sure Arduino is connected.')
        message = 'Can\'t connect to port {}.' \
                  ' You can set the port using the parameter {}.'.format(port, port_param)
        rospy.logwarn(message)
        return

    soundClient.say('hello')
    rospy.loginfo('connected to port {}'.format(ser.name))  # check which port was really used


def stop():
    global ser, state

    if state == DISCONNECTED:
        return

    if ser.is_open:
        ser.close()

    state = DISCONNECTED

    rospy.loginfo('disconnected from the port.')


def open_curtain():
    send_serial_command(b'1')


def close_curtain():
    send_serial_command(b'0')


def stop_curtain_motor():
    send_serial_command(b'2')


def is_connected():
    return True if state == CONNECTED else False


commands = {
    'start': start,
    'stop': stop,
    'open_cl': open_curtain,
    'close_cl': close_curtain,
    'stop_cl_motro': stop_curtain_motor
}


def handle_request(req):
    rospy.logdebug("processing req: {}".format(req))
    if req.command in commands.keys():
        command_func = commands[req.command]
        params = inspect.getargspec(command_func)[0]

        if hasattr(req, 'param') and len(params) > 0:
            command_func(req.param)
        else:
            command_func()

        rospy.logdebug("returning ok")
        return ArduinoResponse('ok')
    else:
        rospy.logdebug("returning command not found")
        return ArduinoResponse('command_not_found')


def curtain_server():
    global ser

    rospy.init_node('arduino_server', log_level=rospy.DEBUG)

    _ = rospy.Service('arduino', Arduino, handle_request)
    rospy.loginfo("Ready to handle parrot commands.")
    rospy.spin()

    stop()


if __name__ == "__main__":
    curtain_server()
