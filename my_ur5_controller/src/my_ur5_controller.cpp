#include "ros/ros.h"
#include "std_msgs/Float64.h"
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

float force = 0.0;
float currentPos[6];
float initialJointPositions[] = {
    0.0, -0.91, 0.32, 0.14, 0.03, 0.0};
float jointLimits[] = {
    2 * M_PI,
    2 * M_PI,
    M_PI,
    2 * M_PI,
    2 * M_PI,
    2 * M_PI,
};

float increment = -0.005;

KDL::JntArray jointPositions(N_JOINTS);

std::string urdfFile = "/home/manuel/ros/hrtask_ws/src/ur5_robot_control/ur_description/urdf/ur5.urdf";

std::string positionTopicNames[] = {
    "/shoulder_pan_joint_controller/command",
    "/shoulder_lift_joint_controller/command",
    "/elbow_joint_controller/command",
    "/wrist_1_joint_controller/command",
    "/wrist_2_joint_controller/command",
    "/wrist_3_joint_controller/command",
};

void commandCallback(const std_msgs::Float64::ConstPtr &msg)
{
    force = msg->data;
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
        jointPositions(i) = msg->position[mapIndex[i]];
        //std::cout << msg->name[i] << std::endl;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ur5_controller_node");
    ros::NodeHandle node;
    KDL::Tree tree;
    KDL::Chain chain;

    ros::Subscriber subCommand = node.subscribe("command", 1000, commandCallback);
    ros::Subscriber subState = node.subscribe("/joint_states", 1000, stateCallback);

    std::vector<ros::Publisher> positionPublishers;
    ros::Publisher positionPublisher;
    for (int i = 0; i < N_JOINTS; i++)
    {
        positionPublisher = node.advertise<std_msgs::Float64>(positionTopicNames[i], 1);
        positionPublishers.push_back(positionPublisher);
    }

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
    std_msgs::Float64 jointPos;
    //ros::Rate rate(0.5);
    ros::Rate rate(30);

    for (int i = 0; i < N_JOINTS; i++)
    {
        jointPos.data = initialJointPositions[i];
        positionPublishers[i].publish(jointPos);
        q_min(i) = -jointLimits[i];
        q_max(i) = jointLimits[i];
    }
    rate.sleep();

    while (ros::ok)
    {

        // Do Forward Kinematic
        KDL::ChainFkSolverPos_recursive fkSolverPos(chain);
        KDL::Frame posFrame;
        fkSolverPos.JntToCart(jointPositions, posFrame);

        //increment in cartesian x direction

        if (posFrame.p(0) > 0.6)
        {
            increment = -0.005;
        }
        else if (posFrame.p(0) < 0.3)
        {
            increment = 0.005;
        }
        posFrame.p(0) += increment;

        ROS_INFO("X: %f, Y: %f, Z: %f", posFrame.p(0), posFrame.p(1), posFrame.p(2));

        //Do Inverse Kinematic
        KDL::ChainIkSolverVel_pinv ikSolverVel(chain);
        KDL::ChainIkSolverPos_NR_JL ikSolverPos(chain, q_min, q_max, fkSolverPos, ikSolverVel, 100, 1e-6);

        KDL::JntArray q_dest(N_JOINTS);
        ikSolverPos.CartToJnt(jointPositions, posFrame, q_dest);

        //std::cout << "New Joint Values:" << std::endl;
        for (int i = 0; i < N_JOINTS; i++)
        {
            //std::cout << q_dest(i) << std::endl;
            jointPos.data = q_dest(i);
            //jointPos.data = (float)q_dest(i);
            positionPublishers[i].publish(jointPos);
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}