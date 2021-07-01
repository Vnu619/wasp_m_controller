#!/usr/bin/env python

import rospy
import time
import bmsXnergy
import sys
import math
import tf
import numpy as np
from geometry_msgs.msg import Twist
from roboclaw import Roboclaw
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from time import sleep
from std_msgs.msg import *
from sensor_msgs.msg import Imu


# slave addresses
flag=0

# kinematic parameters
xdot = 0
ydot = 0
thetadot = 0
R = 0.100 # Wheel radius 100mm
B = 0.387 # Distance between two wheels 387mm
#a = 0.3
#b = 0.25
loop_rate =5


class WaspController(object):

	def __init__(self):
		self.wheel_odom_pub = rospy.Publisher('wheel_odom', Odometry, queue_size=10)
		self.target_speed_pub = rospy.Publisher('target_speed',Twist, queue_size = 10)
		#rospy.Subscriber("/mobile_base/commands/velocity", Twist, self.loop)
		rospy.Subscriber("cmd_vel", Twist, self.loop)
		rospy.Subscriber('/speed_fl', Int32, self.fl_speed)
		rospy.Subscriber('/speed_fr', Int32, self.fr_speed)
		rospy.Subscriber("/vectornav/IMU", Imu, self.imuCallback)

		self.x_ = 0
		self.y_ = 0
		self.z_ =0
		self.th = 0
		self.prev_time = rospy.Time.now()
		self.odomBroadcaster = tf.TransformBroadcaster()
		self.imu_msg=Imu()
		self.gear_ratio=50
		self.ydot=0
		self.xdot=0
		self.thetadot=0
		self.roll = 0
		self.pitch = 0
		self.prev_yaw = 0
		self.imu_flag=0
		self.fl_msg=0
		self.fr_msg=0
		self.bl_msg=0
		self.br_msg=0
		self.velx=0
		self.vely=0


	def fl_speed(self, msg):
		self.fl_msg= msg.data


	def fr_speed(self, msg):
		self.fr_msg=msg.data


	def imuCallback(self, msg):
		self.imu_msg=msg
		if (self.imu_flag==0):
			(self.roll, self.pitch, self.prev_yaw) = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
		#print("prev_yaw",self.prev_yaw)
		self.imu_flag=1


	def forwardKinematics(self):
		ydot=self.ydot
		xdot=-self.xdot
		thetadot=self.thetadot
		phi_fl = 1 / R * (-ydot - xdot + (a + b) * thetadot) * 60 / 3.14 * 50
		phi_fr = 1 / R * (-ydot + xdot + (a + b) * thetadot) * 60 / 3.14 * 50
		#print phi_fl, phi_fr, phi_br, phi_bl

		target_speed = Twist()
		target_speed.linear.x = phi_fl
		target_speed.linear.y = phi_fr
		target_speed.angular.y = 0
		target_speed.angular.z = 0

		curr_time = rospy.Time.now()
		dt = ((curr_time - self.prev_time).to_sec() )
		v_fl_real = float(self.fl_msg)/60*R*(3.14/50)
		v_fr_real = float(self.fr_msg)/60*R*(3.14/50)
		(r, p, yaw) = tf.transformations.euler_from_quaternion([self.imu_msg.orientation.x, self.imu_msg.orientation.y, self.imu_msg.orientation.z, self.imu_msg.orientation.w])
		delta_th = yaw - self.prev_yaw
		Vx_real = 0.25*(-v_fl_real-v_fr_real)
		Vy_real = -0.25*(-v_fl_real+v_fr_real)
		self.prev_yaw = yaw
		delta_x = (Vx_real * math.cos(-self.th) - Vy_real * math.sin(-self.th)) * dt
		delta_y = (Vx_real * math.sin(-self.th) + Vy_real * math.cos(-self.th)) * dt
		thetadot_real  = delta_th / dt
		self.x_  += delta_x
		self.y_  += delta_y
		self.th += delta_th
		vx = Vx_real
		vy = Vy_real
		vth =thetadot_real
		self.velx=vx
		self.vely=vy

		odom_quat = Quaternion()
		odom_quat = tf.transformations.quaternion_from_euler(0,0,self.th)
		odom = Odometry()
		odom.header.frame_id = "odom"
		odom.child_frame_id = "base_link"
		odom.header.stamp = rospy.Time.now()
		odom.pose.pose.position.x = self.y_
		odom.pose.pose.position.y = self.x_
		odom.pose.pose.position.z = self.z_
		odom.pose.pose.orientation = Quaternion(*odom_quat)
		odom.twist.twist.linear.x = vy
		odom.twist.twist.linear.y = vx
		odom.twist.twist.angular.z = vth
		self.prev_time = curr_time
		self.wheel_odom_pub.publish(odom)
		self.target_speed_pub.publish(target_speed)


	def pause(self):
		self.xdot = 0.0
		self.ydot = 0.0
		self.thetadot = 0.0
		time.sleep(0.3)


	def loop(self,msg):
		#self.pause()
		self.xdot = msg.linear.x
		self.ydot = msg.linear.y
		self.thetadot = -msg.angular.z


def listener():
	rospy.init_node('loop',anonymous = True)
	_object = WaspController()
	rate = rospy.Rate(20)
	#prev_time = rospy.Time.now()
	print ("test ok")
	while not rospy.is_shutdown():

	_object.forwardKinematics()
	#rospy.loginfo("Ok !")
	rate.sleep()
	#rospy.spin()


if __name__=='__main__':
	listener()
