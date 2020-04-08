#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include <sstream>

ros::Publisher joyPub;
ros::Subscriber joySub;

void joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
    geometry_msgs::Twist twist;

    twist.linear.x = joy->axes[0]; // Left Joystic:  left: +x right: -x
    twist.linear.y = joy->axes[1]; // Left Joystic:  up  : +y down : -y
    twist.linear.z = joy->axes[4]; // Right Joystic: up  : +z down : -z
    joyPub.publish(twist);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "commander_node");
    ros::NodeHandle node;
    joyPub = node.advertise<geometry_msgs::Twist>("command", 1);
    joySub = node.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);

    ros::spin();
    return 0;
}
