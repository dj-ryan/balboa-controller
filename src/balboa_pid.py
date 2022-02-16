#!/usr/bin/env python
from __future__ import print_function
from ast import While
from geometry_msgs.msg import Twist
from balboa_core.msg import balboaMotorSpeeds
from balboa_core.msg import balboaLL
import rospy

# Publishers
pubMotor = rospy.Publisher('/motorSpeeds', balboaMotorSpeeds, queue_size=10)

# global vars
currentState = balboaLL()
pVal = 0.00014
targetAngle = 0
motorSpeed = balboaMotorSpeeds()
accuracyTolerance = 100




def correctAngle(targetAngle):
    while (abs(targetAngle - currentState.angleZ) > accuracyTolerance):
        motorSpeed.left = pVal * (targetAngle - currentState.angleZ)
        motorSpeed.right = pVal * (currentState - targetAngle.angleZ)
        pubMotor.publish(motorSpeed)
        



def callbackIMU(data):
    rospy.loginfo("IMU updated")
    rospy.loginfo(data.angleZ)
    currentState = data



def callbackKey(data):
    rospy.loginfo("A key was pressed")
    global targetAngle
    rospy.loginfo("key pressed")
    if (data.angular.z < 0):
        targetAngle = targetAngle - 85500
    elif(data.angular.z > 0):
        targetAngle = targetAngle + 85000
    rospy.loginfo("new angle: %d", targetAngle)
    
    #correctAngle(targetAngle)
    

def pidSubscriptions():
    rospy.init_node('balboa_pid')
    rospy.loginfo("STARTED")
    rospy.Subscriber('/turtle1/cmd_vel', Twist, callbackKey)
    rospy.Subscriber('/balboaLL', balboaLL, callbackIMU)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        rospy.loginfo("while")
        motorSpeed.left = 0
        motorSpeed.right = 0
        pubMotor.publish(motorSpeed)
        rate.sleep()

if __name__ == "__main__":
    pidSubscriptions()