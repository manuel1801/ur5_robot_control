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

using namespace KDL;
bool use_robot = false;

int main(int argc, char **argv)
{
    KDL::Chain chain;

    if (!use_robot)
    {
        //Simple robot arm with two segments.KDL::Chain chain;
        chain.addSegment(Segment(Joint(Joint::RotZ),
                                 Frame(Vector(1.0, 0.0, 0.0))));
        //chain.addSegment(Segment(Joint(Joint::RotY),
        //                         Frame(Vector(0.0, 1.0, 0.0))));
        chain.addSegment(Segment(Joint(Joint::RotZ),
                                 Frame(Vector(1.0, 0.0, 0.0))));
    }
    else
    {
        ros::init(argc, argv, "my_test_controller");
        ros::NodeHandle node;
        std::string robot_desc_string;
        node.param("robot_description", robot_desc_string, std::string());

        // KDL::Tree kdl_tree;
        // if (!kdl_parser::treeFromString(robot_desc_string, kdl_tree))
        // {
        //     ROS_ERROR("Failed to construct kdl tree");
        //     return false;
        // }

        KDL::Tree kdl_tree;
        if (!kdl_parser::treeFromFile("/home/manuel/ros/hrtask_ws/src/universal_robot/my_ur5_controller/src/ur5.urdf", kdl_tree))
        {
            ROS_ERROR("Failed to construct kdl tree");
            return false;
        }

        unsigned int nj = kdl_tree.getNrOfJoints();
        unsigned int js = kdl_tree.getNrOfSegments();
        ROS_INFO("NrOfJoints =%d   ||  NrOfSegments= %d \n", nj, js);

        KDL::SegmentMap::const_iterator rootSegment = kdl_tree.getRootSegment();
        kdl_tree.getChain("base_link", "ee_link", chain);
    }

    /*for (int i = 0; i < 6; i++)
    {
        Segment s = chain.getSegment(i);
        std::cout << s.getJoint().getName() << std::endl;
        std::cout << s.getJoint().JointAxis().x()
                  << s.getJoint().JointAxis().y()
                  << s.getJoint().JointAxis().z() << std::endl;

        std::cout << s.getJoint().JointAxis().x()
                  << s.getJoint().JointAxis().y()
                  << s.getJoint().JointAxis().z() << std::endl;
    }*/

    // Create solver based on kinematic chain
    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);

    // Create joint array
    KDL::JntArray jointpositions = JntArray(chain.getNrOfJoints());

    ChainIkSolverVel_pinv iksolver1v(chain); //Inverse velocity solver
    ChainIkSolverPos_NR iksolver1(chain, fksolver, iksolver1v, 100, 1e-6);

    //Creation of jntarrays:
    JntArray q(chain.getNrOfJoints());
    JntArray q_init(chain.getNrOfJoints());

    //Set destination frame
    Frame F_dest = Frame(Vector(1.0, 1.0, 0.0));
    int ret = iksolver1.CartToJnt(q_init, F_dest, q);

    std::cout << ret << std::endl;
    std::cout << "status: " << ret << q(0) << q(1) << std::endl;

    return 0;
}