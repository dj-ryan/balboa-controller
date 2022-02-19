#!/usr/bin/env python
from __future__ import print_function
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist
from balboa_core.msg import balboaMotorSpeeds

#from beginner_tutorials.srv import AddTwoInts,AddTwoIntsResponse
import rospy

pubMotor = rospy.Publisher('/motorSpeeds', balboaMotorSpeeds, queue_size = 10)

def callback(data):
    rospy.loginfo("data x: %f, data z: %f", data.linear.x, data.linear.z)

    motorSpeed = balboaMotorSpeeds()
    motorSpeed.left = 100
    motorSpeed.right = 100
    pubMotor.publish(motorSpeed)

def teleop_listener():
    rospy.init_node('teleop_listener')
    rospy.loginfo("STARTED")
    rospy.Subscriber('/turtle1/cmd_vel', Twist, callback)
    
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == "__main__":
    teleop_listener()
