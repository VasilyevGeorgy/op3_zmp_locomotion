#ifndef OP3_KDL_MOVE_PELVIS_H
#define OP3_KDL_MOVE_PELVIS_H

#include <cmath>
#include <stdint.h>
#include <string>
#include <vector>
#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <eigen3/Eigen/Eigen>
#include <kdl/joint.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainiksolverpos_lma.hpp>

#define JOINT_NUM (6)
#define D2R (M_PI/180.0)
#define R2D (180.0/M_PI)

// pattern name_ for not KDL types, name for their KDL analog

class op3_kdl_move_pelvis
{
public:
  op3_kdl_move_pelvis();
  virtual ~op3_kdl_move_pelvis();

  void setJointPosition(Eigen::VectorXd rleg_joint_position_,Eigen::VectorXd lleg_joint_position_); // l&r joint position as input
  void solveForwardKinematics(KDL::Frame &body_pose); // FK output for legs wrt current joint position
  bool solveInverseKinematics(KDL::Frame pelvis_pose,
                                Eigen::VectorXd &rleg_des_joint_pos_, Eigen::VectorXd &lleg_des_joint_pos_); // pelvis pose as input, joint position as output

private:
  ros::NodeHandle node;

  //KDL::Frame pelvis_pose;
  KDL::JntArray rleg_joint_position;
  KDL::JntArray lleg_joint_position;

//Joint limits
  KDL::JntArray min_joint_position_limit;
  KDL::JntArray max_joint_position_limit;

//Right leg
  KDL::Chain rleg_chain;

  KDL::ChainFkSolverPos_recursive *rleg_fk_solver;
  KDL::ChainIkSolverVel_pinv *rleg_ik_vel_solver;
  KDL::ChainIkSolverPos_LMA *rleg_ik_pos_solver;

//Left leg
  KDL::Chain lleg_chain;

  KDL::ChainFkSolverPos_recursive *lleg_fk_solver;
  KDL::ChainIkSolverVel_pinv *lleg_ik_vel_solver;
  KDL::ChainIkSolverPos_LMA *lleg_ik_pos_solver;

//IK
  Eigen::VectorXd rleg_joint_position_, lleg_joint_position_;

  KDL::JntArray rleg_des_joint_pos;
  KDL::JntArray lleg_des_joint_pos;


};

#endif // OP3_KDL_MOVE_PELVIS_H
