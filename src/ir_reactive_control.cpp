#include "ros/ros.h"
#include "std_msgs/String.h"

#include "balboa_core/balboaLL.h"
#include "geometry_msgs/Twist.h"

int loopCount = 0;

float range = 0;

void callback(const balboa_core::balboaLL &data)
{
    ROS_INFO("sensor: %d", data.rangeSensor);

    range = data.rangeSensor;
}

int main(int argc, char **argv)
{

    ROS_INFO("balba_controller/ir_reactiveControl node starting......");

    ros::init(argc, argv, "ir_reactive_control");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("balboaLL", 1000, callback);

    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("ir_reactive_control/cmd_vel", 1000);

    ros::Rate loop_rate(10);

    // ros::spin();

    while (ros::ok())
    {
        geometry_msgs::Twist move;

        int target = 300;

        int moveValue = target - range;

        ROS_INFO("move: %d", moveValue);

        move.linear.x = moveValue;

        pub.publish(move);

        // if (range > 200) // go backwards
        // {
        //     move.linear.x = range;
        //     pub.publish(move);
        // }
        // else if (range > 300) // go forwards
        // {
        //     move.linear.x = -range;
        //     pub.publish(move);
        // } else {
        //     move.linear.x = 0;
        //     pub.publish(move);
        // }

        ros::spinOnce();
        // delay
        loop_rate.sleep();

        ++loopCount;
    }

    return 0;
}