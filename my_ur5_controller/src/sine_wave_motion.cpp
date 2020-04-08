#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include <cmath>

#define N_JOINTS 6

std::string positionTopicNames[] = {
    "/shoulder_pan_joint_controller/command",
    "/shoulder_lift_joint_controller/command",
    "/elbow_joint_controller/command",
    "/wrist_1_joint_controller/command",
    "/wrist_2_joint_controller/command",
    "/wrist_3_joint_controller/command",
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sine_wave_motion");
    ros::NodeHandle node;
    std_msgs::Float64 jointPos;

    std::vector<ros::Publisher> positionPublishers;
    ros::Publisher positionPublisher;
    for (int i = 0; i < N_JOINTS; i++)
    {
        positionPublisher = node.advertise<std_msgs::Float64>(positionTopicNames[i], 1);
        positionPublishers.push_back(positionPublisher);
    }

    ros::Rate rate(20);
    float n = 0;
    float offset = -(M_PI / 2);

    while (ros::ok)
    {
        //Initial Pose
        if (n < 20)
        {
            for (int i = 0; i < N_JOINTS; i++)
            {
                if (i == 1)
                {

                    jointPos.data = offset;
                }
                else
                {
                    jointPos.data = 0;
                }
                //continue;
                positionPublishers[i].publish(jointPos);
            }
        }
        else
        {
            //Sine Wave Pose
            for (int i = 0; i < N_JOINTS; i++)
            {
                if (i == 1)
                {
                    jointPos.data = sin(ros::Time::now().toSec()) + offset;
                }
                else
                {
                    jointPos.data = sin(ros::Time::now().toSec());
                }
                positionPublishers[i].publish(jointPos);
            }
        }

        ros::spinOnce();
        rate.sleep();
        n++;
    }
    return 0;
}