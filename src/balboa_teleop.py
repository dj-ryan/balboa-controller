#!/usr/bin/env python
from __future__ import print_function
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist
from balboa_core.msg import balboaMotorSpeeds


import rospy

pubMotor = rospy.Publisher('/motorSpeeds', balboaMotorSpeeds, queue_size = 10)
motorSpeed = balboaMotorSpeeds()

def callback(data):
    rospy.loginfo("data x: %f, data z: %f", data.linear.x, data.linear.z)

    if (data.linear.x < 0):
        motorSpeed.left = 25
        motorSpeed.right = 25
    elif (data.linear.x > 0):
        motorSpeed.left = -25
        motorSpeed.right = -25
    elif(data.angular.z < 0):
        motorSpeed.left = 25
        motorSpeed.right = -25
    elif(data.angular.z > 0):
        motorSpeed.left = -25
        motorSpeed.right = 25
    else:
        motorSpeed.left = 0
        motorSpeed.right = 0

def teleop_listener():
    rospy.init_node('teleop_listener')
    rospy.loginfo("teleop_listener started...")
    rospy.Subscriber('/turtle1/cmd_vel', Twist, callback)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rospy.loginfo("motor left: %f, right: %f", motorSpeed.left, motorSpeed.right)
        pubMotor.publish(motorSpeed)
        motorSpeed.left = 0
        motorSpeed.right = 0
        rate.sleep()


if __name__ == "__main__":
    teleop_listener()
