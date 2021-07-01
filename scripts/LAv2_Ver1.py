#!/usr/bin/env python
# This code is dedicated for simpler linear actuator control without needed to call function. #
# For codes with functions please refer to roboclaw_linearactuator.py #

from roboclaw import Roboclaw
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import rospy
import time

# linux comport name & baudrate
rc = Roboclaw("/dev/ttyACM0",115200)

# RoboClaw address at 128 decimal, equivalent to 0x80 hexadecimal.
# Set upper limit = 1580 & lower limit = 80 to avoid linear actuator trigger mechanical limit switch.
# Set upper position = 1480 & lower position = 180 to give tolerance as linear actuator does not stop on certain value.
# Set linear actuator initial position = 250 for standby state position.
rc.Open()
address = 0x80
upper_limit = 1180
lower_limit = 80
upper_position = 1080
lower_position = 180
initial_position = 250

# Speed of the linear actuator
# The max. QPPS has been set at 430 in basic micro as it is the max value
# of QPPS when linear actuator is moving in fastest speed.
# As 12V is supplied to roboclaw, hence 12V = 430QPPS.
# Linear actuator works at voltage range 6V to 12V. To gain safety range in working voltage,
# 7V is supplied to linear actuator. Through calculation, 7V equals to 250QPPS.
# Thus speed of linear actuator in rc.SpeedM1 is set at 250.

# globalize variable a and position.
key = 0
position = 0
init_button = 0


# create a node with name "velocity_node"
# this node subscribe to topic /cmd_vel and post
# rospy.spin() to keep this function looping infinitely until a Ctrl+C is triggered.
def listener():
    rospy.init_node('velocity_node', anonymous=True)
    rospy.Subscriber("/cmd_vel", Twist, callback_cmd_vel)
    rospy.Subscriber("post", Int32, callback_position_feedback)
    rospy.spin()


def callback_position_feedback(data):
    global position
    position = data.data
    if lower_limit < position < upper_limit:
        if key > 0.0 and (lower_limit < position < upper_position):
            rc.SpeedM1(address, 250)
            print("The linear actuator is extruding...")
        elif key < 0.0 and (lower_position < position < upper_limit):
            rc.SpeedM1(address, -250)
            print("The linear actuator is retracting...")
        elif key == 0.0:
            rc.SpeedM1(address, 0)
            print("The linear actuator stops.")
        elif position < lower_position:
            rc.SpeedM1(address, 0)
            print("The linear actuator minimum position reached.")
        elif position > upper_position:
            rc.SpeedM1(address, 0)
            print("The linear actuator maximum position reached.")
    else:
        rc.SpeedM1(address, 0)
        time.sleep(2)
        print("The linear actuator is beyond limit. Initialization needed.")

    if init_button > 0.1:
        initialize()
    else:
        rc.SpeedM1(address, 0)


def callback_cmd_vel(msg):
    rospy.loginfo("z-value: %f", msg.linear.x)
    global key
    global init_button
    key = msg.angular.x
    init_button = msg.angular.y


# Initialize function
# Move linear actuator to initial position.
def initialize():
    if rc.ReadEncM1(address)[1] != initial_position:
        if rc.ReadEncM1(address)[1] < initial_position:
            while rc.ReadEncM1(address)[1] < initial_position:
                rc.SpeedM1(address, 250)
                time.sleep(0.1)
                display()
            else:
                rc.SpeedM1(address, 0)
                time.sleep(2)
                print("Initialization done!")
        else:
            while rc.ReadEncM1(address)[1] > initial_position:
                rc.SpeedM1(address, -250)
                time.sleep(0.1)
                display()
            else:
                rc.SpeedM1(address, 0)
                time.sleep(2)
                print("Initialization done!")
    else:
        print ("The linear actuator is ready!")


def display():
    print("Position = ", rc.ReadEncM1(address)[1])


if __name__ == '__main__':
    initialize()
    listener()
