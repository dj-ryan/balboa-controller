#include "ros/ros.h"
#include "std_msgs/String.h"

#include "balboa_core/balboaLL.h"
#include "geometry_msgs/Twist.h"

int loopCount = 0;

float range = 0;

void callback(const balboa_core::balboaLL &data)
{
	//grab range sensor data from balboa robot
    range = data.rangeSensor;
}

int main(int argc, char **argv)
{
	//initiallize node/subscriber/publisher
    ROS_INFO("balba_controller/ir_reactiveControl node starting......");
    ros::init(argc, argv, "ir_reactive_control");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("balboaLL", 1000, callback);
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("ir_reactive_control/cmd_vel", 1000);
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        geometry_msgs::Twist move;

		//set target distance and calculate the distance robot needs to move
        int target = 300;
        int moveValue = target - range;
        move.linear.x = moveValue;

		//publish needed movement to topic for pidLab2.py to use
        pub.publish(move);

        ros::spinOnce();
        loop_rate.sleep();
        ++loopCount;
    }

    return 0;
}