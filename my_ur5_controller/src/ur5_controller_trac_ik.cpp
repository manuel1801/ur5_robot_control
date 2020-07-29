#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include <string>
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#define N_JOINTS 6

float force[] = {0.0, 0.0, 0.0};
float initJointPos[] = {0.0, -M_PI / 2, 0.0, 0.0, 0.0, 0.0};
float rate = 2000;
float scale = 0.004;

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
    force[0] = scale * msg.linear.x;
    force[1] = scale * msg.linear.y;
    force[2] = scale * msg.linear.z;
}

// from Callback:
// [elbow_joint, shoulder_lift_joint, shoulder_pan_joint, wrist_1_joint, wrist_2_joint,wrist_3_joint]

// from urdf:
// [shoulder_pan_joint , shoulder_lift_joint, elbow_joint , wrist_1_joint, wrist_2_joint,wrist_3_joint]
int mapIndex[] = {2, 1, 0, 3, 4, 5};

void stateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    for (int i = 0; i < N_JOINTS; i++)
    {
        jointPosCurrent(i) = msg->position[mapIndex[i]];
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "ur5_controller_trac_ik");
    ros::NodeHandle node;

    std::vector<ros::Publisher> positionPublishers;
    ros::Publisher positionPublisher;
    std_msgs::Float64 jointPosMsg;

    // Subscriber to receive commands for the force in x,y,z
    ros::Subscriber subCommand = node.subscribe("command", 1000, commandCallback);

    // Subscriber to receive current joint positions
    ros::Subscriber subState = node.subscribe("/joint_states", 1000, stateCallback);

    KDL::Chain chain;
    KDL::JntArray ll, ul; //lower joint limits, upper joint limits
    KDL::Frame eefPose;

    TRAC_IK::TRAC_IK tracik_solver("base_link", "ee_link", "robot_description");

    bool valid = tracik_solver.getKDLChain(chain);
    if (!valid)
    {
        ROS_ERROR("There was no valid KDL chain found");
        return 0;
    }
    valid = tracik_solver.getKDLLimits(ll, ul);
    if (!valid)
    {
        ROS_ERROR("There were no valid KDL joint limits found");
        return 0;
    }
    assert(chain.getNrOfJoints() == ll.data.size());
    assert(chain.getNrOfJoints() == ul.data.size());
    ROS_INFO("Using %d joints", chain.getNrOfJoints());

    KDL::JntArray jointPosDest(chain.getNrOfJoints());
    KDL::JntArray q_init(chain.getNrOfJoints());

    KDL::ChainFkSolverPos_recursive fk_solver(chain);

    //Init Joints
    for (uint j = 0; j < q_init.data.size(); j++)
    {
        q_init(j) = (ll(j) + ul(j)) / 2.0;
        jointPosDest(j) = initJointPos[j];
        positionPublisher = node.advertise<std_msgs::Float64>(positionTopicNames[j], 1);
        positionPublishers.push_back(positionPublisher);
    }

    ros::Rate loop_rate(rate);
    int n = 0;

    while (ros::ok)
    {
        // Go to initial Pose
        if (n < rate)
        {
            for (int i = 0; i < chain.getNrOfJoints(); i++)
            {
                jointPosMsg.data = jointPosDest(i);
                positionPublishers[i].publish(jointPosMsg);
            }
        }
        else
        {
            // Do Forward Kinematic
            fk_solver.JntToCart(jointPosCurrent, eefPose);

            // Increment the current position by force value from command topic
            eefPose.p(0) += force[0];
            eefPose.p(1) += force[1];
            eefPose.p(2) += force[2];

            //Do Inverse Kinematic
            if (tracik_solver.CartToJnt(jointPosCurrent, eefPose, jointPosDest) >= 0)
            {
                for (int i = 0; i < chain.getNrOfJoints(); i++)
                {
                    jointPosMsg.data = jointPosDest(i);
                    positionPublishers[i].publish(jointPosMsg);
                }
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
        n++;
    }
    return 0;
}