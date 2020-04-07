#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include <string>
#include <kdl/chain.hpp>
//#include <kdl/chainfksolver.hpp>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

#include <kdl/tree.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treeiksolvervel_wdls.hpp>
#include <kdl/treeiksolverpos_nr_jl.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>

#define PI 3.1416
#define N_JOINTS 6

//using namespace KDL;

// void goCallback(const std_msgs::Bool &b)
// {
//     pub.publish(msg);
//     std::cout << "command executed" << std::endl;
// }

float force = 0.0;
float currentPos[6];
KDL::JntArray q_init(N_JOINTS);

std::string positionTopicNames[] = {
    "/shoulder_pan_joint_controller/command",
    "/shoulder_lift_joint_controller/command",
    "/elbow_joint_controller/command",
    "/wrist_1_joint_controller/command",
    "/wrist_2_joint_controller/command",
    "/wrist_3_joint_controller/command",
};

void forceCallback(const std_msgs::Float64::ConstPtr &msg)
{
    //ROS_INFO("received x: %.2f,  y: %.2f, z: %.2f", msg.linear.x, msg.linear.y, msg.linear.z);
    force = msg->data;
    //hier const x (kraft) auf aktuelle joint state (cartesian) aufaddieren
}

// JntArray parseJointStates(std::vector<std::string> names, std::vector<double> positions)
// {
//     JntArray q_temp(6);
//     int count = 0;
//     bool parsed = false;
//     for (unsigned int i = 0; i < names.size(); i++)
//     {
//         if (strncmp(names[i].c_str(), "arm_", 4) == 0)
//         {
//             q_temp(count) = positions[i];
//             count++;
//             parsed = true;
//         }
//     }
//     if (!parsed)
//         return q_last;
//     q_last = q_temp;
//     //ROS_INFO("CurrentConfig: %f %f %f %f %f %f %f", q_temp(0), q_temp(1), q_temp(2), q_temp(3), q_temp(4), q_temp(5), q_temp(6));
//     if (!started)
//     {
//         JntArray zero(7);
//         //sendVel(zero);
//         VirtualQ = q_temp;
//         started = true;
//         last = ros::Time::now();

//         ROS_INFO("Starting up controller with first configuration: %f %f %f", q_temp(0), q_temp(1), q_temp(2));
//     }
//     return q_temp;
// }

void stateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    // ROS_INFO("Received: ", msg->position[0]);
    //std::cout << "posJ1: " << msg->position[0] << "  posJ2: " << msg->position[1] << std::endl;
    for (int i = 0; i < N_JOINTS; i++)
    {
        q_init(i) = msg->position[i];
    }

    // std::vector<std::string> names = msg->name;
    // std::vector<double> positions = msg->position;
    // q = parseJointStates(names, positions);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_ur5_controller");
    ros::NodeHandle node;
    std::string robot_desc_string;
    node.param("robot_description", robot_desc_string, std::string());

    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromString(robot_desc_string, kdl_tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }

    unsigned int nj = kdl_tree.getNrOfJoints();
    unsigned int js = kdl_tree.getNrOfSegments();
    ROS_INFO("NrOfJoints =%d   ||  NrOfSegments= %d \n", nj, js);

    KDL::Chain kdl_chain;
    KDL::SegmentMap::const_iterator rootSegment = kdl_tree.getRootSegment();
    kdl_tree.getChain("base_link", "ee_link", kdl_chain);

    KDL::ChainFkSolverPos_recursive fkSolverPos(kdl_chain);
    KDL::ChainIkSolverVel_pinv ikSolverVel(kdl_chain);
    KDL::ChainIkSolverPos_NR ikSolverPos(kdl_chain, fkSolverPos, ikSolverVel);

    for (int i = 0; i < N_JOINTS; i++)
    {
        //q_init(i) = msg->position[i];
        q_init(i) = 0.0;
    }

    KDL::JntArray q_dest(N_JOINTS);
    KDL::Frame p_dest(KDL::Vector(0, 0, 1));

    int ret = ikSolverPos.CartToJnt(q_init, p_dest, q_dest);
    ROS_INFO("IK Solver RetValue: %d.", ret);

    ros::Subscriber subForce = node.subscribe("force", 1000, forceCallback);
    ros::Subscriber subState = node.subscribe("/joint_state_controller", 1000, stateCallback);

    std::vector<ros::Publisher> positionPublishers;
    ros::Publisher positionPublisher;
    for (int i = 0; i < N_JOINTS; i++)
    {
        positionPublisher = node.advertise<std_msgs::Float64>(positionTopicNames[i], 1);
        positionPublishers.push_back(positionPublisher);
    }

    std_msgs::Float64 jointPos;
    ros::Rate rate(0.5);
    int n = 0;
    float pos[] = {0.0, PI / 2.0, PI, (3.0 / 2.0) * PI};

    while (ros::ok)
    {
        for (int i = 0; i < N_JOINTS; i++)
        {
            if (i == 0)
            {
                jointPos.data = pos[n % 4];
            }
            else
            {
                jointPos.data = 0.0;
            }
            positionPublishers[i].publish(jointPos);
        }
        n++;
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}