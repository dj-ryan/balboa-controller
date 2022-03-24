#!/usr/bin/env python
from __future__ import print_function
from balboa_core.msg import balboaMotorSpeeds
from balboa_core.msg import balboaLL
from std_msgs.msg import Int32
import rospy

# Declare and initialize variables 
motor = balboaMotorSpeeds()

targetAngle = 0
targetDistance = 0

# PID constants 
pValAng = 0.0001
dValAng = 0.0000001
pValDis = 0.01
dValDis = 0.0005

initFlag = 0
initIMU = balboaLL()

oldAngError = 0
oldDisError = 0

# PID loop when new commands received 
def callbackIMU(msg):

    # Initialize variables
    global initFlag
    global targetAngle
    global targetDistance
    global oldAngError
    global oldDisError
    
    # Grab encoder state to initialize target values at current state
    if(initFlag == 0):
        initIMU = msg
        initFlag = 1
        targetAngle = initIMU.angleX
        targetDistance = initIMU.distanceRight

    # PID error calculation
    angError = (targetAngle - msg.angleX)
    disError = (targetDistance - msg.distanceRight)

    rospy.loginfo("targetDistance: %d", targetDistance);
    # rospy.loginfo("targetDistance: %d | currentDistance: %d | targetAngle: %d | currentAngle: %d", targetDistance, msg.distanceRight, targetAngle, msg.angleX)
    # rospy.loginfo("disErrorVal: %d | angErrorVal: %d", disError, angError)
    
    # PID calc for angle and distance
    angPidVal = (pValAng * angError) + (dValAng * (angError - oldAngError)) 
    disPidVal = (pValDis * disError) + (dValDis * (disError - oldDisError))
    
    # Publish PID value to motors

    #rospy.loginfo("motor power val: %d", disPidVal)
    motor.right = angPidVal + disPidVal
    motor.left = -angPidVal + disPidVal

    # Update error variables     
    oldAngError = angError
    oldDisError = disError

# Set new distance target from published data
def callbackDrive(msg):
    global targetDistance 
    targetDistance = targetDistance + msg.data

# Set new angle target from published data
def callbackTurn(msg):
    global targetAngle
    targetAngle = targetAngle + msg.data

def pidSubscriptions():

    # Initialize node
    rospy.init_node('pidMap')

    # State node has started on console 
    rospy.loginfo("STARTED")

    # Declare publishers and subscribers
    rospy.Subscriber('/mapDrive', Int32, callbackDrive, queue_size = 15)
    rospy.Subscriber('/mapTurn', Int32, callbackTurn, queue_size = 15)
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
        # motor.left = 0
        # motor.right = 0

        # Sleep
        rate.sleep()

if __name__ == "__main__":
    pidSubscriptions()

