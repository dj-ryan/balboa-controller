#include "ros/ros.h"
#include "balboa_core/balboaLL.h"
#include "std_msgs/Int32.h"

#include <cmath>

#include <map>
#include <string>

std::map<int, int> exampleRow = {{0, 0}, {1, 0}, {2, 1}, {3, 1}, {4, 2}, {5, 2}, {6, 2}, {7, 1}, {8, 1}, {9, 0}, {10, 0}};

std::string printMapRow(std::map<int, int> row, int i)
{
	std::string rowOutput = "row " + std::to_string(i) + " => ";
	for (const auto &dataPoint : row)
	{
		if (dataPoint.second == 2)
		{
			rowOutput.append("0");
		}
		else if (dataPoint.second == 2)
		{
			rowOutput.append("o");
		}
		else
		{
			rowOutput.append("_");
		}
	}

	return rowOutput;
}

void printMap(std::vector<std::map<int, int>> rows)
{
	int i = 0;
	for (std::map<int, int> r : rows)
	{
		ROS_INFO(printMapRow(r, i).c_str());
		i++;
	}
}

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

	// Example row print 
	ROS_INFO(printMapRow(exampleRow, 0).c_str());

	// Declare subscribers and publishers
	ros::Subscriber sub = node.subscribe("balboaLL", 1000, callbackIMU);

	ros::Publisher pubDrive = node.advertise<std_msgs::Int32>("mapDrive", 1000);
	ros::Publisher pubTurn = node.advertise<std_msgs::Int32>("mapTurn", 1000);

	// Set standard distance targets and turn targets
	std_msgs::Int32 targetDrive;
	std_msgs::Int32 targetAngle;

	// Initialize map grid variables
	int snake[200];

	int totalRows = 4;
	int rowLength = 3000;
	int doneOnce = 0;
	int rightTurn = 96000;
	int leftTurn = -96000;
	int goToNextRow = 400;
	int finishLine = 0;

	// Set loop rate
	ros::Rate loop_rate(0.5);

	// Main node loop
	while (ros::ok())
	{
		// Drive robot through totalRows, and then stop
		if (doneOnce == 0)
		{
			doneOnce = 1;

			// Drive robot through totalRows
			for (int i = 0; i < totalRows; i++)
			{
				// Add robot's current encoder amount to the "finish line" or total row distance
				finishLine = robotData.distanceRight + rowLength;

				// Remember robot's current orientation
				// targetAngle.data = robotData.angleX;
				// targetDrive.data = robotData.distanceRight;

				// Drive the first row
				while (robotData.distanceRight < finishLine)
				{
					targetDrive.data = 500;
					pubDrive.publish(targetDrive);

					// ROS_INFO("target: %d - Sens1: %d | Sens2: %d | Sens4: %d | Sens5: %d", targetDrive.data, robotData.IRsensor1, robotData.IRsensor2, robotData.IRsensor4, robotData.IRsensor5);
					ROS_INFO("target: %d", targetDrive.data);
					// snake[j - 1] = robotData.IRsensor1;

					ros::spinOnce();
					loop_rate.sleep();
				}

				ROS_INFO(" == FINISHED ROW [%d] == | finishline: %d", i, finishLine);

				// Make first 90 deg turn out of row, alternate left and right turns
				if (i % 2 == 0)
				{
					targetAngle.data = rightTurn;
					pubTurn.publish(targetAngle);
					while (abs(robotData.angleX) < abs(targetAngle.data))
					{
						ros::spinOnce();
						// Do nothing
					}
				}
				else
				{
					targetAngle.data = leftTurn;
					pubTurn.publish(targetAngle);
					while (abs(robotData.angleX) > abs(targetAngle.data))
					{
						ros::spinOnce();
						// Do nothing
					}
				}

				// Update right encoder after turn
				targetDrive.data = robotData.distanceRight;
				ROS_INFO(" == Completed first turn | update encoder: %d == ", targetDrive.data);

				// Drive to next row
				targetDrive.data = goToNextRow;
				pubDrive.publish(targetDrive);
				while (abs(robotData.distanceRight) < abs(targetDrive.data))
				{
					ros::spinOnce();
					// Do nothing
				}

				ROS_INFO(" == Arrived at next row == ");

				// Make second 90 deg turn into row, alternate left and right turns
				if (i % 2 == 0)
				{
					targetAngle.data = leftTurn;
					pubTurn.publish(targetAngle);
					while (abs(robotData.angleX) > abs(targetAngle.data))
					{
						ros::spinOnce();
						// Do nothing
					}
				}
				else
				{
					targetAngle.data = rightTurn;
					pubTurn.publish(targetAngle);
					while (abs(robotData.angleX) < abs(targetAngle.data))
					{
						ros::spinOnce();
						// Do nothing
					}
				}

				ROS_INFO(" == Completed second turn! == ");
			}
		}

		ROS_INFO(" == FINISHED MAPPING PROCEDURE == ");

		// Spin loop and sleep
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}