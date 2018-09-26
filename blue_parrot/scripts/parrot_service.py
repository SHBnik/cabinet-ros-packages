#!/usr/bin/env python
import rospy
import serial
import time
import threading
import inspect
from parrot.srv import *
from serial import SerialException
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from os.path import expanduser


DEFAULT_PORT = '/dev/ttyUSB0'
P_PORT = 'port'
TEN_SECONDS = 10
CONNECTED = 'connected'
DISCONNECTED = 'disconnected'

ser = serial.Serial()
lock = threading.Lock()
soundClient = SoundClient()
state = DISCONNECTED

sounds_folder_path = expanduser("~") + '/parrot_files/sounds/'
sounds = [
    'interactive/Afarin.wav',
    'interactive/AreBarandeShodim.wav',
    'interactive/AreFekreKhoobieh.wav',
    'interactive/Bale.wav',
    'interactive/AreManEshtebhGoftam.wav',
    'interactive/asabanimishe.wav',
    'interactive/Asabani Shod.wav',
    'interactive/avalman.wav',
    'interactive/avaltobegoo.wav',
    'interactive/barandehshodim.wav',
    'interactive/moafegham.wav',
    'interactive/basheshorookon.wav',
    'interactive/BebinEshtebahGoftiSekehKam.wav',
    'interactive/behemonsekehmide.wav',
    'interactive/salaam.wav',
    'interactive/Snore.wav',
    'interactive/berimmarhalebad.wav',
    'interactive/beshinpishema.wav',
    'interactive/dorostgofti.wav',
    'interactive/beshinsandali.wav',
    'interactive/biabazibob.wav',
    'interactive/chap.wav',
    'interactive/rast.wav',
    'interactive/vasat.wav',
    'interactive/EsmetChieh.wav',
    'interactive/halanobatemane.wav',
    'interactive/havasetojamkon.wav',
    'interactive/hooradorostbood.wav',
    'interactive/hoselam.wav',
    'interactive/khastehshodam.wav',
    'interactive/kheilibobdoost.wav',
    'interactive/khoobi.wav',
    'interactive/mamnoontokhoobi.wav',
    'interactive/marhalehtamoom.wav',
    'interactive/nabayaddad.wav',
    'interactive/saketbashi.wav',
    'interactive/sekehamonziadshod.wav',
    'interactive/zoodbashbegoo.wav',
    'animals/Ahoo.wav',
    'animals/Asb.wav',
    'animals/Babr.wav',
    'animals/Gav.wav',
    'animals/Gorbeh.wav',
    'animals/Gorg.wav',
    'animals/Gosfand.wav',
    'animals/Khargoosh.wav',
    'animals/Khoroos.wav',
    'animals/Morgh.wav',
    'animals/Palang.wav',
    'animals/Roobah.wav',
    'animals/Sag.wav',
    'animals/Shir.wav',
    'fruits/Ananas.wav',
    'fruits/Angoor.wav',
    'fruits/Gilas.wav',
    'fruits/Golabi.wav',
    'fruits/Hendoone.wav',
    'fruits/Holoo.wav',
    'fruits/Khiar.wav',
    'fruits/Kivi.wav',
    'fruits/Moz.wav',
    'fruits/Porteghal.wav',
    'fruits/Sib.wav',
    'fruits/Zardaloo.wav',
    'bodies/Abroo.wav',
    'bodies/Angosht.wav',
    'bodies/Bazoo.wav',
    'bodies/Cheshm.wav',
    'bodies/Dast.wav',
    'bodies/Gardan.wav',
    'bodies/Goosh.wav',
    'bodies/Lab.wav',
    'bodies/Pa.wav',
    'bodies/Sar.wav',
    'bodies/Shekam.wav'
]


def send_serial_command(command, sleep_time=0.0):
    with lock:
        try:
            ser.write(command)
            time.sleep(sleep_time)
        except SerialException:
            rospy.loginfo('SerialException on write {}'.format(SerialException))


wings_open = False
interception = True


def motor1():
    send_serial_command('\x01\xff', 3)
    
def dance():
    send_serial_command('\x01\xff', 3)
    time.sleep(0.2)
    send_serial_command('\x01\xff', 3)


def stop_motor1():
    send_serial_command('\x05')


def stop_motor2():
    send_serial_command('\x06')


def open_eyes():
    send_serial_command('\x0d')


def close_eyes():
    send_serial_command('\x0e')


def blink_eyes():
    send_serial_command('\x20\x0d')


def open_mouth():
    send_serial_command('\x11')


def close_mouth():
    send_serial_command('\x0c')


def open_and_close_mouth():
    send_serial_command('\x0b')


def move_left():
    global interception
    if not interception:
        start_position()
    send_serial_command('\x01\xff', 0.8)
    send_serial_command('\x05')
    interception = False


def move_right():
    global interception
    if not interception:
        start_position()
    send_serial_command('\x01\xff', 1.7)
    send_serial_command('\x05')
    interception = False


def open_wings():
    global wings_open, interception
    if not wings_open:
        if not interception:
            start_position()

        time.sleep(0.1)
        send_serial_command('\x02\xff', 0.256)
        send_serial_command('\x05')
        interception = False
        wings_open = True


def close_wings():
    global wings_open, interception
    if wings_open:
        time.sleep(0.3)
        send_serial_command('\x02\xff', 0.256)
        send_serial_command('\x05')
        interception = True
        wings_open = False


def start_position():
    global wings_open, interception
    send_serial_command('\x01\xff', 3)
    interception = True
    wings_open = False


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
    ser.baudrate = 4800
    ser.parity = serial.PARITY_NONE

    try:
        ser.open()
        state = CONNECTED
    except SerialException:
        state = DISCONNECTED
        soundClient.say('I can\'t connect to the port. Make sure the robot is connected.')
        message = 'Can\'t connect to port {}.' \
                  ' You can set the port using the parameter {}.'.format(port, port_param)
        rospy.logwarn(message)
        return

    soundClient.say('hello')
    rospy.loginfo('connected to port {}'.format(ser.name))  # check which port was really used

    send_serial_command('\x01\xff')
    stop_motor1()


def stop():
    global ser, state

    if state == DISCONNECTED:
        return

    if ser.is_open:
        ser.close()

    state = DISCONNECTED

    rospy.loginfo('disconnected from the port.')


def is_connected():
    return True if state == CONNECTED else False


def play_voice(number):
    if 0 <= number < len(sounds):
        soundClient.playWave('{}{}'.format(sounds_folder_path, sounds[number]))
        time.sleep(0.5)
        open_and_close_mouth()
        time.sleep(0.5)
        open_and_close_mouth()

def shutup():
    soundClient.stopWave('{}{}'.format(sounds_folder_path, sounds[number]))
    close_mouth()


commands = {
    'start': start,
    'stop': stop,
    'motor1': motor1,
    'stop_motor1': stop_motor1,
    'stop_motor2': stop_motor2,
    'open_eyes': open_eyes,
    'close_eyes': close_eyes,
    'blink_eyes': blink_eyes,
    'open_mouth': open_mouth,
    'close_mouth': close_mouth,
    'open_and_close_mouth': open_and_close_mouth,
    'move_left': move_left,
    'move_right': move_right,
    'open_wings': open_wings,
    'close_wings': close_wings,
    'play_voice': play_voice,
    'shutup':shutup
}


def handle_parrot(req):
    rospy.logdebug("processing req: {}".format(req))
    if req.command in commands.keys():
        command_func = commands[req.command]
        params = inspect.getargspec(command_func)[0]

        if hasattr(req, 'param') and len(params) > 0:
            command_func(req.param)
        else:
            command_func()

        rospy.logdebug("returning ok")
        return ParrotResponse('ok')
    else:
        rospy.logdebug("returning command not found")
        return ParrotResponse('command_not_found')


def parrot_server():
    global ser

    rospy.init_node('parrot_server', log_level=rospy.DEBUG)

    _ = rospy.Service('parrot', Parrot, handle_parrot)
    rospy.loginfo("Ready to handle parrot commands.")
    rospy.spin()

    stop()


if __name__ == "__main__":
    parrot_server()
