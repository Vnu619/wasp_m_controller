#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from roboclaw import Roboclaw

rc = Roboclaw("/dev/ttyACM0",115200)

rc.Open()
address = 0x80


def talker():
    pub = rospy.Publisher('post', Int32, queue_size=10)
    rospy.init_node('position_feedback', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        #position_int =0
        position_int = rc.ReadEncM1(address)[1]
        rospy.loginfo(position_int)
        pub.publish(position_int)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
