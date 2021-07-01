#!/usr/bin/env python
import rospy
import time
import math
import tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import *
from roboclaw import Roboclaw

rc = Roboclaw("/dev/ttyACM0",115200)

rc.Open()
address = 0x80
upper_limit = 1180
lower_limit = 80
upper_position = 1080
lower_position = 180
initial_position = 250

# Define kinematics parameters
xdot = 0
thetadot = 0
R = 0.100   # Wheel radius 100mm
B = 0.345   # Distance between wheels 345mm
L = 0.300   # The length of the robot base 300mm
loop_rate = 5

flag = 0
#file1 = open("/home/wasp/Desktop/velocities.txt", "a")

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
        else:
            while rc.ReadEncM1(address)[1] > initial_position:
                rc.SpeedM1(address, -250)
                time.sleep(0.1)
            else:
                rc.SpeedM1(address, 0)
                time.sleep(2)
    else:
        print("The linear actuator is ready!")


def display():
    print("Position = ", rc.ReadEncM1(address)[1])


class WASPMini(object):
    def __init__(self):
        self.target_speed_pub = rospy.Publisher('target_speed', Twist, queue_size=10)
        self.wheel_odom_pub = rospy.Publisher('wheel_odom', Odometry, queue_size=10)
        rospy.Subscriber('/cmd_vel', Twist, self.loop)
        rospy.Subscriber('/speed_fl', Int32, self.fl_speed)
        rospy.Subscriber('/speed_fr', Int32, self.fr_speed)
        rospy.Subscriber('/vectornav/IMU', Imu, self.imucallback)
        rospy.Subscriber("/post", Int32, self.callback_position_feedback)

        self.x_ = 0
        self.y_ = 0
        self.z_ = 0
        self.th = 0
        self.prev_time = rospy.Time.now()
        self.odomBroadcaster = tf.TransformBroadcaster()
        self.imu_msg = Imu()
        self.gear_ratio = 50
        self.xdot = 0
        self.thetadot = 0
        self.roll = 0
        self.pitch = 0
        self.prev_yaw = 0
        self.imu_flag = 0
        self.fl_msg = 0
        self.fr_msg = 0
        self.velx = 0
        self.vely = 0
        self.key = 0
        self.position = 0
        self.init_button = 0

    def fl_speed(self, msg):
        self.fl_msg = msg.data

    def fr_speed(self, msg):
        self.fr_msg = msg.data

    def imucallback(self, msg):
        self.imu_msg = msg
        if self.imu_flag == 0:
            (self.roll, self.pitch, self.prev_yaw) = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientaiton.w])
            self.imu_flag = 1

    def forwardkinematics(self):
        xdot = self.xdot
        thetadot = self.thetadot
        # Speed unit conversion from m/s to rpm, from cmd_vel to motor driver
        #phi_fl = ((xdot / R) + (B * thetadot * 0.5)) * (60 / (2 * 3.14)) * 50
        #phi_fr = -((xdot / R) - (B * thetadot * 0.5)) * (60 / (2 * 3.14)) * 50
        phi_fl = (50 / R) * (xdot + (B / 2) * thetadot) * (60 / (2 * 3.14))
        phi_fr = -(50 / R) * ((xdot) - (B / 2) * thetadot) * (60 / (2 * 3.14))
        #print(phi_fl, -phi_fr)
        # define target_speed
        target_speed = Twist()
        target_speed.linear.x = phi_fl
        target_speed.linear.y = phi_fr
        #target_speed.linear.x = 2500
        #target_speed.linear.y = 2500
        target_speed.angular.y = 0
        target_speed.angular.z = 0
        self.target_speed_pub.publish(target_speed)
        #define dt
        curr_time = rospy.Time.now()
        dt = ((curr_time - self.prev_time).to_sec())
        # feedback velocity from motor driver, conversion to m/s
        v_fl_real = (float(self.fl_msg) * ((2 * 3.14) / 60)) * R / 50
        v_fr_real = -(float(self.fr_msg) * ((2 * 3.14) / 60)) * R / 50
        #print(v_fr_real, v_fr_real)
        (r, p, yaw) = tf.transformations.euler_from_quaternion([self.imu_msg.orientation.x, self.imu_msg.orientation.y, self.imu_msg.orientation.z, self.imu_msg.orientation.w])
        #delta_th = yaw - self.prev_yaw
        self.prev_yaw = yaw
        # robot velocity = 0.5 * (left wheel velocity + right wheel velocity)
        Vx_real = (v_fl_real + v_fr_real) * 0.5
        #thetadot_real = delta_th / dt
        #thetadot_real = 0.5 / (0.3 + B) * (v_fl_real - v_fr_real)
        thetadot_real = (v_fr_real - v_fl_real) / B
        print(Vx_real, thetadot_real)
        delta_x = (Vx_real * math.cos(self.th)) * dt
        delta_y = (Vx_real * math.sin(self.th)) * dt
        delta_th = thetadot_real * dt
        #print(delta_x, delta_y)
        #self.x_ += delta_x
        #self.y_ += delta_y
        #self.th += delta_th
        #vx = Vx_real
        #vth = thetadot_real

        #L_pose = [str(v_fl_real), ",", str(v_fr_real) + "\n"]
        #file1.writelines(L_pose)

        if ((max(v_fl_real, v_fr_real) > 1) or (
                min(v_fl_real, v_fr_real) < -1)):
            self.x_ += 0.0
            self.y_ += 0.0
            self.th += 0.0
            vx = 0.0
            vth = 0.0
        else:

            self.x_ += delta_x
            self.y_ += delta_y
            self.th += delta_th
            vx = Vx_real
            vth = thetadot_real
        self.velx = vx
        odom_quat = Quaternion()
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
        odom = Odometry()
        odom.header.frame_id = 'Odom'
        odom.child_frame_id = 'base_link'
        odom.header.stamp = rospy.Time.now()
        odom.pose.pose.position.x = self.x_
        odom.pose.pose.position.y = self.y_
        odom.pose.pose.position.z = self.z_
        odom.pose.pose.orientation = Quaternion(*odom_quat)
        odom.twist.twist.linear.x = vx
        #odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth
        self.prev_time = curr_time
        self.wheel_odom_pub.publish(odom)

    def loop(self, msg):
        self.key = msg.angular.x
        self.init_button = msg.angular.y
        self.xdot = msg.linear.x
        #self.ydot = msg.linear.y
        self.thetadot = msg.angular.z

    def callback_position_feedback(self, data):
        self.position = data.data
        if (lower_limit < self.position < upper_limit) and (self.init_button != 0.1):
            if self.key > 0.0 and (lower_limit < self.position < upper_position):
                rc.SpeedM1(address, 250)
#                print("The linear actuator is extruding...")
            elif self.key < 0.0 and (lower_position < self.position < upper_limit):
                rc.SpeedM1(address, -250)
#                print("The linear actuator is retracting...")
            elif self.key == 0.0:
                rc.SpeedM1(address, 0)
#                print("The linear actuator stops.")
            elif self.position < lower_position:
                rc.SpeedM1(address, 0)
#                print("The linear actuator minimum position reached.")
            elif self.position > upper_position:
                rc.SpeedM1(address, 0)
#                print("The linear actuator maximum position reached.")
            elif (lower_limit > self.position > upper_limit) and (self.init_button != 0.1):
                rc.SpeedM1(address, 0)
                time.sleep(2)
#               print("The linear actuator is beyond limit. Initialization needed.")
        elif self.init_button == 0.1:
            initialize()


def listener():
    rospy.init_node('loop', anonymous=True)
    _object = WASPMini()
    rate = rospy.Rate(20)
    print("Test ok!")
    while not rospy.is_shutdown():
        _object.forwardkinematics()
        rate.sleep()


if __name__ == '__main__':
    initialize()
    listener()
