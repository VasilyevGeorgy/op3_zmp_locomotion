#ifndef PLANAR_MAN_H
#define PLANAR_MAN_H

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

#define JOINT_NUM (3)
#define D2R (M_PI/180.0)
#define R2D (180.0/M_PI)

//#define JOINT_NUM (3)
//#define D2R (M_PI/180.0)

class Planar_man {

public:
  Planar_man();
  virtual ~Planar_man();

  std::vector<double_t> solveIK(Eigen::Vector3d, Eigen::Vector3d);
  std::vector<double_t> solveFK(Eigen::Vector3d);
  std::vector<double_t> solveFK();
  void setInitialPosition(double_t,double_t,double_t);

private:
  ros::NodeHandle node;

  KDL::JntArray init_joint_position;

  KDL::Chain m_chain;

  std::vector<double> min_position_limit;
  std::vector<double> max_position_limit;

  KDL::JntArray min_joint_position_limit;
  KDL::JntArray max_joint_position_limit;

  KDL::ChainFkSolverPos_recursive *m_fk_solver_;
  KDL::ChainIkSolverVel_pinv *m_ik_vel_solver_;
  KDL::ChainIkSolverPos_NR_JL *m_ik_pos_solver_;

  KDL::Frame m_tip_pose; //FK_pose
  std::vector<double_t> FK_output;

  KDL::Frame tip_desired_pose; //IK_position
  bool is_initialized;
  KDL::JntArray desired_joint_position;
  std::vector<double_t> IK_pos_output;

  KDL::Twist des_cart_vel; //IK_velocities
  KDL::JntArray jnt_vel;
  std::vector<double_t> IK_vel_output;

};

#endif // PLANAR_MAN_H
