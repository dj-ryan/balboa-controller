#!/usr/bin/env python
from __future__ import print_function
from balboa_core.msg import balboaMotorSpeeds
from balboa_core.msg import balboaLL
from std_msgs.msg import Int64
import rospy

# Declare and initialize variables 
motor = balboaMotorSpeeds()

targetAngle = 0
targetDistance = 0

pValAng = 0.0001
dValAng = 0.00000001
pValDis = 0.015
dValDis = 0.0001

# pValAng = rospy.get_param('/pidLab1/pValAng')
# dValAng = rospy.get_param('/pidLab1/dValAng')
# pValDis = rospy.get_param('/pidLab1/pValDis')
# dValDis = rospy.get_param('/pidLab1/dValDis')

initFlag = 0
initIMU = balboaLL()

oldAngError = 0
oldDisError = 0

# PID loop when new commands received 
def callbackIMU(data):

    # Initialize variables
    global initFlag
    global targetAngle
    global targetDistance
    global oldAngError
    global oldDisError
    
    # Grab encoder state to initialize target values at current state
    if(initFlag == 0):
        initIMU = data
        initFlag = 1
        targetAngle = initIMU.angleX
        targetDistance = initIMU.distanceRight

    # PID error calculation
    angError = (targetAngle - data.angleX)
    disError = (targetDistance - data.distanceRight)
    
    # PID calc for angle and distance
    angPidVal = (pValAng * angError) + (dValAng * (angError - oldAngError)) 
    disPidVal = (pValDis * disError) + (dValDis * (disError - oldDisError))
    
    # Publish PID value to motors
    motor.right = angPidVal + disPidVal
    motor.left = -angPidVal + disPidVal

    # Update error variables     
    oldAngError = angError
    oldDisError = disError

# Set new distance target from published data
def callbackDrive(data):
    global targetDistance 
    targetDistance = data

# Set new angle target from published data
def callbackTurn(data):
    global targetAngle
    targetAngle = data

def pidSubscriptions():

    # Initialize node
    rospy.init_node('pidMap')

    # State node has started on console 
    rospy.loginfo("STARTED")

    # Declare publishers and subscribers
    rospy.Subscriber('/mapDrive', Int64, callbackDrive)
    rospy.Subscriber('/mapTurn', Int64, callbackTurn)
    rospy.Subscriber('/balboaLL', balboaLL, callbackIMU)
    pubMotor = rospy.Publisher('/motorSpeeds', balboaMotorSpeeds, queue_size = 10)

    # Set loop rate
    rate = rospy.Rate(10)
    
    # Main node loop
    while not rospy.is_shutdown():

        # State node has begun main loop
        rospy.loginfo("while")

        # Publish latest motor speeds 
        pubMotor.publish(motor)

        # Set motor speeds equal to zero when no new commands
        motor.left = 0
        motor.right = 0

        # Sleep
        rate.sleep()

if __name__ == "__main__":
    pidSubscriptions()
