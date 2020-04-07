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
float increment = -0.1;

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
    //ROS_INFO("received x: %.2f,  y: %.2f, z: %.2f", msg.linear.x, msg.linear.y, msg.linear.z);
    force = msg->data;
    //hier const x (kraft) auf aktuelle joint state (cartesian) aufaddieren
}

void stateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    // ROS_INFO("Received: ", msg->position[0]);
    //std::cout << "posJ1: " << msg->position[0] << "  posJ2: " << msg->position[1] << std::endl;
    //for (int i = 0; i < N_JOINTS; i++)
    //{
    //    q_init(i) = msg->position[i];
    //}
    //jointPositions

    // std::vector<std::string> names = msg->name;
    // std::vector<double> positions = msg->position;
    // q = parseJointStates(names, positions);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ur5_controller_node");
    ros::NodeHandle node;
    KDL::Tree tree;
    KDL::Chain chain;

    ros::Subscriber subCommand = node.subscribe("command", 1000, commandCallback);
    ros::Subscriber subState = node.subscribe("/joint_state_controller", 1000, stateCallback);

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
    ros::Rate rate(0.5);
    rate.sleep();

    for (int i = 0; i < N_JOINTS; i++)
    {
        jointPos.data = initialJointPositions[i];
        positionPublishers[i].publish(jointPos);
        jointPositions(i) = initialJointPositions[i]; //nur in callback
        q_min(i) = -6.28;
        q_max(i) = 6.28;
    }

    // // Do Forward Kinematic
    // KDL::ChainFkSolverPos_recursive fkSolverPos(chain);
    // KDL::Frame posFrame;
    // fkSolverPos.JntToCart(jointPositions, posFrame);
    // ROS_INFO("Forward Kinematic Result x: %f z: %f z: %f",
    //          posFrame.p(0), posFrame.p(1), posFrame.p(2));

    // //increment in cartesian z direction
    // posFrame.p(2) += increment;

    // ROS_INFO("Position after increment x: %f z: %f z: %f",
    //          posFrame.p(0), posFrame.p(1), posFrame.p(2));

    // //Do Inverse Kinematic
    // KDL::ChainIkSolverVel_pinv ikSolverVel(chain);
    // KDL::ChainIkSolverPos_NR_JL ikSolverPos(chain, q_min, q_max, fkSolverPos, ikSolverVel, 100, 1e-6);

    // KDL::JntArray q_dest(N_JOINTS);
    // ikSolverPos.CartToJnt(jointPositions, posFrame, q_dest);

    // std::cout << "New Joint Values:" << std::endl;
    // for (int i = 0; i < N_JOINTS; i++)
    // {
    //     std::cout << q_dest(i) << std::endl;
    // }

    //return 0;

    int n = 0;

    while (ros::ok)
    {

        // Do Forward Kinematic
        KDL::ChainFkSolverPos_recursive fkSolverPos(chain);
        KDL::Frame posFrame;
        fkSolverPos.JntToCart(jointPositions, posFrame);
        ROS_INFO("Forward Kinematic Result x: %f z: %f z: %f",
                 posFrame.p(0), posFrame.p(1), posFrame.p(2));

        //increment in cartesian z direction
        posFrame.p(2) += increment;
        //hier frame from command topic callback
        //     jointPos from stateCB

        ROS_INFO("Position after increment x: %f y: %f z: %f",
                 posFrame.p(0), posFrame.p(1), posFrame.p(2));

        //Do Inverse Kinematic
        KDL::ChainIkSolverVel_pinv ikSolverVel(chain);
        KDL::ChainIkSolverPos_NR_JL ikSolverPos(chain, q_min, q_max, fkSolverPos, ikSolverVel, 100, 1e-6);

        KDL::JntArray q_dest(N_JOINTS);
        ikSolverPos.CartToJnt(jointPositions, posFrame, q_dest);

        std::cout << "New Joint Values:" << std::endl;
        for (int i = 0; i < N_JOINTS; i++)
        {
            std::cout << q_dest(i) << std::endl;
            jointPos.data = q_dest(i);
            //jointPos.data = (float)q_dest(i);
            positionPublishers[i].publish(jointPos);
        }

        if (n % 10 == 0)
        {
            increment *= -1.0;
        }
        ros::spinOnce();
        rate.sleep();
        n++;
    }
    return 0;
}