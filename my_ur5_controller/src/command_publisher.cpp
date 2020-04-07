#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <sstream>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "command_publisher");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::Float64>("command", 1000);
    ros::Rate loop_rate(10);

    std_msgs::Float64 msg;
    //float positions[] = {-2.0, -1.5, -1.0, -0.5, 0, 0.5, 1.0, 1.5, 2.0};
    //stattdessen zb const x = 1, y,z=0

    int count = 0;
    while (ros::ok())
    {
        //msg.data = positions[count % 9];
        msg.data = 1.0;
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}
