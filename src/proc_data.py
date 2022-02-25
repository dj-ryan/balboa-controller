#!/usr/bin/env python
from __future__ import print_function
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist
from balboa_core.msg import balboaMotorSpeeds

import rospy

pubMotor = rospy.Publisher('/motorSpeeds', balboaMotorSpeeds, queue_size = 10)
motor = balboaMotorSpeeds()

def callback(data):
    rospy.loginfo("data x: %f, data z: %f", data.linear.x, data.linear.z)

    if (data.linear.x < 0):
        motor.left = 10
        motor.right = 10
    elif (data.linear.x > 0):
        motor.left = -10
        motor.right = -10
    elif(data.angular.z < 0):
        motor.left = 10
        motor.right = -10
    elif(data.angular.z > 0):
        motor.left = -10
        motor.right = 10
    else:
        motor.left = 0
        motor.right = 0

def teleop_listener():
    rospy.init_node('teleop_listener')
    rospy.loginfo("STARTED")
    rospy.Subscriber('/turtle1/cmd_vel', Twist, callback)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rospy.loginfo("in WHILE: %f", motor.left)
        pubMotor.publish(motor)
        motor.left = 0
        motor.right = 0
        rate.sleep()

if __name__ == "__main__":
    teleop_listener()



