#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include <string>
#include <kdl/chain.hpp>
//#include <kdl/chainfksolver.hpp>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

#include <kdl/tree.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treeiksolvervel_wdls.hpp>
#include <kdl/treeiksolverpos_nr_jl.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>

std::string urdfFile = "/home/manuel/ros/hrtask_ws/src/ur5_robot_control/ur_description/urdf/ur5.urdf";

using namespace KDL;
bool use_robot = true;

int main(int argc, char **argv)
{
    KDL::Chain chain;
    KDL::Tree tree;

    ros::init(argc, argv, "my_test_controller");
    ros::NodeHandle node;

    if (!kdl_parser::treeFromFile(urdfFile, tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }

    //tree.getChain("base_link", "tool0", chain);
    tree.getChain("world", "ee_link", chain);
    int nrOfJ = chain.getNrOfJoints();

    // Create solver based on kinematic chain
    KDL::ChainFkSolverPos_recursive fksolver(chain);
    // Create joint array
    KDL::JntArray jointPositions = JntArray(nrOfJ);
    KDL::JntArray j_min = JntArray(nrOfJ);
    KDL::JntArray j_max = JntArray(nrOfJ);
    for (int i = 0; i < nrOfJ; i++)
    {
        //hier dann aus callback
        //jointPositions(i) = 0;
        j_min(i) = -6.28;
        j_max(i) = 6.28;
    }
    jointPositions(0) = 0.0;
    jointPositions(1) = -0.91;
    jointPositions(2) = 0.32;
    jointPositions(3) = 0.14;
    jointPositions(4) = 0.03;
    jointPositions(5) = 0.0;

    // Forward Kinematic
    KDL::Frame cartPosFrame;

    // Calculate forward position kinematics
    bool kinematics_status = fksolver.JntToCart(jointPositions, cartPosFrame);

    std::cout << "STATUS: " << kinematics_status << " Cartesian Pos from FK:" << cartPosFrame.p << std::endl;

    //cartPosFrame.p(0) = 0.681554;
    //cartPosFrame.p(1) = 0.19145;
    cartPosFrame.p(2) -= 0.358777;

    std::cout << "new Cartesian Pos" << cartPosFrame.p << std::endl;

    // Vector tmp(0.0, 0.0, 0.0);
    // Vector cartPosVector = cartPosFrame * tmp;
    // std::cout << cartPosVector << std::endl;

    ChainIkSolverVel_pinv iksolver1v(chain);
    ChainIkSolverPos_NR_JL iksolver1(chain, j_min, j_max, fksolver, iksolver1v, 100, 1e-6);
    //ChainIkSolverPos_NR iksolver1(chain, fksolver, iksolver1v, 100, 1e-6);

    //Creation of jntarrays:
    KDL::JntArray q_dest(chain.getNrOfJoints());

    //Set destination frame
    int ret = iksolver1.CartToJnt(jointPositions, cartPosFrame, q_dest);

    std::cout << "status: " << ret << std::endl;
    for (int i = 0; i < nrOfJ; i++)
    {
        std::cout << q_dest(i) << std::endl;
    }

    return 0;
}