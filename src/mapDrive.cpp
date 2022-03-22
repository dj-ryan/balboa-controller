#include "ros/ros.h"
#include "balboa_core/balboaLL.h"
#include "std_msgs/Int32.h"

#include <cmath>

// Declare and initialize variables
balboa_core::balboaLL robotData;

// Store robot IR data when received
void callbackIMU(const balboa_core::balboaLL &data)
{
	// Update robot IR data to use later
	robotData = data;
}

int main(int argc, char **argv)
{
	// State node has started on console
	ROS_INFO("STARTED");

	// Initialize node
	ros::init(argc, argv, "mapDrive");
	ros::NodeHandle node;

	// Declare subscribers and publishers
	ros::Subscriber sub = node.subscribe("balboaLL", 1000, callbackIMU);

	ros::Publisher pubDrive = node.advertise<std_msgs::Int32>("mapDrive", 1000);
	ros::Publisher pubTurn = node.advertise<std_msgs::Int32>("mapTurn", 1000);

	// Set standard distance targets and turn targets
	std_msgs::Int32 targetDrive;
	std_msgs::Int32 targetAngle;
	targetDrive.data = robotData.distanceRight;
	targetAngle.data = robotData.angleX;

	// Initialize map grid variables
	// int average;
	// int snake[240];

	// Set loop rate
	ros::Rate loop_rate(10);

	int totalRows = 4;
	int rowLength = 2000;
	int doneOnce = 0;
	int rightTurn = 96000;
	int leftTurn = -96000;
	int goToNextRow = 400;

	// Main node loop
	while (ros::ok())
	{

		// Publish target values
		// pub.publish();

		// for (int i = 1; i < 48; i++)
		// {

		// 	// Store reflectance data point
		// 	average = (((robotData.IRsensor1) + (robotData.IRsensor2) + (robotData.IRsensor3) + (robotData.IRsensor4) + (robotData.IRsensor5)) / 5);
		// 	snake[i - 1] = average;

		// 	// Publish target distance values
		// 	pubDrive.publish(targetDrive);

		// 	// TEST print the reflectance data point
		// 	ROS_INFO("Avg Reflectance: %d", snake[i - 1]);
		// }

		// targetDrive.data += 50;
		// ROS_INFO("new target: %d", (int)targetDrive.data);

		if (doneOnce == 0)
		{
			doneOnce = 1;
			for (int i = 0; i < totalRows; i++)
			{
				// add finishline delta
				int finishLine = robotData.distanceRight + rowLength;
				// remember current orintation
				targetAngle.data = robotData.angleX;
				targetDrive.data = robotData.distanceRight;

				for (targetDrive.data = robotData.distanceRight; targetDrive.data < finishLine; targetDrive.data += 20)
				{
					pubDrive.publish(targetDrive);
					ROS_INFO("target: %d - Sens1: %d | Sens2: %d | Sens4: %d | Sens5: %d", targetDrive.data, robotData.IRsensor1, robotData.IRsensor2, robotData.IRsensor4, robotData.IRsensor5);
					ros::spinOnce();
					loop_rate.sleep();
				}
				ROS_INFO(" == FINISHED ROW [%d] == | finishline: %d", i, finishLine);

				// make first 90 deg turn out of row
				if (i % 2 == 0)
				{
					targetAngle.data += rightTurn;
					pubTurn.publish(targetAngle);
					while (abs(robotData.angleX) < abs(targetAngle.data))
					{
						ros::spinOnce();
						// do nothing
					}
				}
				else
				{
					targetAngle.data += leftTurn;
					pubTurn.publish(targetAngle);
					while (abs(robotData.angleX) > abs(targetAngle.data))
					{
						ros::spinOnce();
						// do nothing
					}
				}

				// update right encoder after turn
				targetDrive.data = robotData.distanceRight;
				ROS_INFO(" == Compleated first turn | update encoder: %d == ", targetDrive.data);

				// drive to next row
				targetDrive.data += goToNextRow;
				pubDrive.publish(targetDrive);
				while (abs(robotData.distanceRight) < abs(targetDrive.data))
				{
					ros::spinOnce();
					// do nothing
				}

				ROS_INFO(" == Arrived at next row == ");

				// make next turn into next row
				if (i % 2 == 0)
				{
					targetAngle.data += rightTurn;
					pubTurn.publish(targetAngle);
					while (abs(robotData.angleX) < abs(targetAngle.data))
					{
						ros::spinOnce();
						// do nothing
					}
				}
				else
				{
					targetAngle.data += leftTurn;
					pubTurn.publish(targetAngle);
					while (abs(robotData.angleX) > abs(targetAngle.data))
					{
						ros::spinOnce();
						// do nothing
					}
				}

				ROS_INFO(" == Compleated second turn! == ");
			}
		}

		ROS_INFO(" == FINISHED MAPPING PROCEDURE == ");

		// Spin loop and sleep
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}