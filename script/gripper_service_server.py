#! /usr/bin/env python

import rospy
import serial
from faraday_gripper.srv import *
import serial.tools.list_ports

scalar = 120/18.5  # Scales the input to the servo values, value = 6.486

#try:
#    ser = serial.Serial('/dev/ttyACM0', 9600)  # Initialize serial port 1
#    print 'Port = /dev/ttyACM0'
#except:
#    try:
#        ser = serial.Serial('/dev/ttyACM1', 9600)  # Initialize serial port 2
#        print 'Port = /dev/ttyACM1'
#    except:
#        try:
#            ser = serial.Serial('/dev/ttyACM2', 9600)  # Initialize serial port 3
#            print 'Port = /dev/ttyACM2'
#        except:
#            print 'Port with Arduino not found'
#            exit()

## The folloing part is looking for an Arduino Serial port
## Source: https://stackoverflow.com/questions/24214643/python-to-automatically-select-serial-ports-for-arduino

arduino_ports = [
    p.device
    for p in serial.tools.list_ports.comports()
    if 'Arduino' in p.description
]
if not arduino_ports:
    print "Not Arduino found!"
    exit()
if len(arduino_ports) > 1:
    print "Multiple Arduinos found - using the first"

ser = serial.Serial(arduino_ports[0])

if "Arduino" in p[1]:
    print "This is an Arduino!"
    print p[0]

# use "Serial.parseInt()" in Arduino to read the integers

def control_gripper(request_gripper):
    print "Requesting for the gripper"
    print "Size to open is equal to ",request_gripper.size_open, " cm.", 
    command = scalar * request_gripper.size_open

    if command < 0:
        print "Too small!"
        command = 0
    elif command > 120:
        print "Too big!"
        command = 120

    command = int(120 - command)
    try:
        ser.write(str(command))
        print "Sended: command =  ", command
        return True
    except:
        print "Not sended because of unknown reason."
        return False

def control_led(request_led):
    print "Requesting for the LED"
    if request_led.LED_on is True:
        print "Turning LED on"
        command = 200
        ser.write(str(command))
        print "Sended: command = "
        print request_led.LED_on
        return True
    elif request_led.LED_on is False:
        print "Turning LED off"
        command = 210
        ser.write(str(command))
        print "Sended: command = "
        print request_led.LED_on
        return True
    else:
        print "Not sended because of an unknown reason."
        return False


def gripper_server():
    rospy.init_node('gripper_service_server')
    gripper_service = rospy.Service('gripper_control', GripperControl, control_gripper)
    led_service = rospy.Service('led_control', LEDControl, control_led)
    print "Ready to execute task."

    rospy.spin()

if __name__ == "__main__":
    gripper_server()