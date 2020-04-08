#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include <string>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/tree.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>

#define N_JOINTS 6

float force[] = {0.0, 0.0, 0.0};
KDL::JntArray jointPosCurrent(N_JOINTS);

std::string positionTopicNames[] = {
    "/shoulder_pan_joint_controller/command",
    "/shoulder_lift_joint_controller/command",
    "/elbow_joint_controller/command",
    "/wrist_1_joint_controller/command",
    "/wrist_2_joint_controller/command",
    "/wrist_3_joint_controller/command",
};

void commandCallback(const geometry_msgs::Twist &msg)
{
    float scale = 0.1; // scale values between -0.1 and 0.1
    force[0] = scale * msg.linear.x;
    force[1] = scale * msg.linear.y;
    force[2] = scale * msg.linear.z;
}

// from Callback:
// [elbow_joint, shoulder_lift_joint, shoulder_pan_joint, wrist_1_joint, wrist_2_joint,wrist_3_joint]

// from urdf:
// [shoulder_pan_joint , shoulder_lift_joint, elbow_joint , wrist_1_joint, wrist_2_joint,wrist_3_joint]
int mapIndex[] = {2, 1, 0, 4, 5, 6};

void stateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    for (int i = 0; i < N_JOINTS; i++)
    {
        jointPosCurrent(i) = msg->position[mapIndex[i]];
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ur5_controller_node");
    ros::NodeHandle node;

    KDL::Tree tree;
    KDL::Chain chain;
    KDL::JntArray jointPosDest(N_JOINTS);
    KDL::JntArray q_min(N_JOINTS);
    KDL::JntArray q_max(N_JOINTS);
    KDL::Frame posFrame;

    std::vector<ros::Publisher> positionPublishers;
    ros::Publisher positionPublisher;
    std_msgs::Float64 jointPosMsg;

    float offsetJ2 = -(M_PI / 2);

    // Subscriber to receive commands for the force in x,y,z
    ros::Subscriber subCommand = node.subscribe("command", 1000, commandCallback);

    // Subscriber to receive current joint positions
    ros::Subscriber subState = node.subscribe("/joint_states", 1000, stateCallback);

    // Get KDL Tree and Chain from urdf File
    std::string robot_desc_string;
    node.param("robot_description", robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string, tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }
    tree.getChain("world", "ee_link", chain);

    for (int i = 0; i < N_JOINTS; i++)
    {
        // Create Publisher to pusblish joint angle values to the pos controller
        positionPublisher = node.advertise<std_msgs::Float64>(positionTopicNames[i], 1);
        positionPublishers.push_back(positionPublisher);

        switch (i)
        {
        case 1:
            jointPosDest(i) = offsetJ2;
            q_min(i) = -(2 * M_PI);
            q_max(i) = (2 * M_PI);
            break;
        case 2:
            jointPosDest(i) = 0;
            q_min(i) = -M_PI;
            q_max(i) = M_PI;
            break;
        default:
            jointPosDest(i) = 0;
            q_min(i) = -(2 * M_PI);
            q_max(i) = (2 * M_PI);
            break;
        }
    }

    ros::Rate rate(20);
    int n = 0;

    while (ros::ok)
    {
        // Go to initial Pose
        if (n < 20)
        {
            for (int i = 0; i < N_JOINTS; i++)
            {
                jointPosMsg.data = jointPosDest(i);
                positionPublishers[i].publish(jointPosMsg);
            }
        }
        else
        {
            // Do Forward Kinematic
            KDL::ChainFkSolverPos_recursive fkSolverPos(chain);
            fkSolverPos.JntToCart(jointPosCurrent, posFrame);

            // Increment the current position by force value from command topic
            posFrame.p(0) += force[0];
            posFrame.p(1) += force[1];
            posFrame.p(2) += force[2];

            //Do Inverse Kinematic
            KDL::ChainIkSolverVel_pinv ikSolverVel(chain);
            KDL::ChainIkSolverPos_NR_JL ikSolverPos(chain, q_min, q_max, fkSolverPos, ikSolverVel, 100, 1e-6);
            ikSolverPos.CartToJnt(jointPosCurrent, posFrame, jointPosDest);

            // Publish new Joint Values
            for (int i = 0; i < N_JOINTS; i++)
            {
                jointPosMsg.data = jointPosDest(i);
                positionPublishers[i].publish(jointPosMsg);
            }
        }

        ros::spinOnce();
        rate.sleep();
        n++;
    }
    return 0;
}