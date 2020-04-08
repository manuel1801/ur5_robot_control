#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include <kdl/chain.hpp>
#include <string>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

#include <kdl/tree.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>

#define N_JOINTS 6

float force[] = {0.0, 0.0, 0.0};
// float initialJointPositions[] = {
//     0.0, -0.91, 0.32, 0.14, 0.03, 0.0};

// float initialCartPositions[] = {
//     0.01, 0.19, 0.91};

float increment = -0.01;

KDL::JntArray jointPosCurrent(N_JOINTS);

std::string urdfFile = "/home/manuel/ros/hrtask_ws/src/ur5_robot_control/ur_description/urdf/ur5.urdf";

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
    force[0] = msg.linear.x;
    force[1] = msg.linear.y;
    force[2] = msg.linear.z;
}

//from Callback joint states
//[elbow_joint, shoulder_lift_joint, shoulder_pan_joint, wrist_1_joint, wrist_2_joint,wrist_3_joint]

//in urdf:
// [shoulder_pan_joint , shoulder_lift_joint, elbow_joint , wrist_1_joint, wrist_2_joint,wrist_3_joint]
int mapIndex[] = {2, 1, 0, 4, 5, 6};

void stateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    for (int i = 0; i < N_JOINTS; i++)
    {
        jointPosCurrent(i) = msg->position[mapIndex[i]];
        //std::cout << msg->name[i] << std::endl;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ur5_controller_node");
    ros::NodeHandle node;
    KDL::Tree tree;
    KDL::Chain chain;
    KDL::JntArray jointPosDest(N_JOINTS);
    KDL::Frame posFrame;
    float offset = -(M_PI / 2);

    ros::Subscriber subCommand = node.subscribe("/command", 1000, commandCallback);
    ros::Subscriber subState = node.subscribe("/joint_states", 1000, stateCallback);

    std::vector<ros::Publisher> positionPublishers;
    ros::Publisher positionPublisher;

    // Get KDL Tree and Chain from urdf File
    if (!kdl_parser::treeFromFile(urdfFile, tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }
    tree.getChain("world", "ee_link", chain);

    // Create initial Joint Array and publish to Controller
    KDL::JntArray q_min(N_JOINTS);
    KDL::JntArray q_max(N_JOINTS);
    std_msgs::Float64 jointPosMsg;

    for (int i = 0; i < N_JOINTS; i++)
    {
        positionPublisher = node.advertise<std_msgs::Float64>(positionTopicNames[i], 1);
        positionPublishers.push_back(positionPublisher);

        switch (i)
        {
        case 1:
            jointPosDest(i) = offset;
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

            // posFrame.p(0) += force[0];
            // posFrame.p(1) += force[1];
            // posFrame.p(2) += force[2];

            if (posFrame.p(2) < 0.4)
            {
                increment = 0.01;
            }
            else if (posFrame.p(2) > 0.7)
            {
                increment = -0.01;
            }
            posFrame.p(2) += increment;

            //Do Inverse Kinematic
            KDL::ChainIkSolverVel_pinv ikSolverVel(chain);
            KDL::ChainIkSolverPos_NR_JL ikSolverPos(chain, q_min, q_max, fkSolverPos, ikSolverVel, 100, 1e-6);

            ikSolverPos.CartToJnt(jointPosCurrent, posFrame, jointPosDest);

            //std::cout << "New Joint Values:" << std::endl;
            for (int i = 0; i < N_JOINTS; i++)
            {
                //std::cout << q_dest(i) << std::endl;
                jointPosMsg.data = jointPosDest(i);
                //jointPos.data = (float)q_dest(i);
                positionPublishers[i].publish(jointPosMsg);
            }
        }

        ros::spinOnce();
        rate.sleep();
        n++;
    }
    return 0;
}