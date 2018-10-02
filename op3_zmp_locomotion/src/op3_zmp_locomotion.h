#ifndef OP3_ZMP_LOCOMOTION_H
#define OP3_ZMP_LOCOMOTION_H

#include <string>
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
#include "std_msgs/Float64.h"

#define JOINT_NUM (6)
#define D2R (M_PI/180.0)
#define R2D (180.0/M_PI)

// pattern name_ for not KDL types, name for their KDL analog

class op3_zmp_locomotion
{
public:
  op3_zmp_locomotion();
  virtual ~op3_zmp_locomotion();

//Walk parameters
  double step_length; // mm
  double clearance; // mm
  double step_duration; // ms
  double pelvis_position_z;
  std::string init_leg;


  void op3_right_leg(KDL::Frame pelvis_pose, KDL::Frame rfoot_pose);
  void op3_left_leg(KDL::Frame pelvis_pose, KDL::Frame lfoot_pose);
  void initialize(KDL::Frame pelvis_pose,
                    KDL::Frame rfoot_pose,
                    KDL::Frame lfoot_pose); //create chains

  void setJointPosition(Eigen::VectorXd rleg_joint_position_,Eigen::VectorXd lleg_joint_position_); // l&r joint position as input


 // bool solveIKPelvisToFoot(KDL::Frame pelvis_des_pose,
 //                           Eigen::VectorXd &leg_des_joint_pos_);
 // bool solveIKFootToPelvis(KDL::Frame foot_des_pose,
 //                           Eigen::VectorXd &leg_des_joint_pos_);

  bool walk(int step_num);

  bool setInitPose(KDL::Frame pelvis_des_pose, Eigen::VectorXd &rleg_cur_joint_pos_, Eigen::VectorXd &lleg_cur_joint_pos_);

  bool moveLeftLeg(KDL::Frame lleg_des_pose, Eigen::VectorXd &lleg_des_joint_pos_);
  bool moveCOMToLeftLeg(KDL::Frame pelvis_des_pose, Eigen::VectorXd &lleg_des_joint_pos_);

  void comTranslation(Eigen::VectorXd &rleg_cur_joint_pos_, Eigen::VectorXd &lleg_cur_joint_pos_);




  bool moveRightLeg(KDL::Frame rleg_des_pose, Eigen::VectorXd &rleg_des_joint_pos_);
  bool moveCOMToRightLeg(KDL::Frame pelvis_des_pose, Eigen::VectorXd &rleg_des_joint_pos_);

  void quit(); //delete chains


private:
  ros::NodeHandle node;

  KDL::JntArray rleg_joint_position;
  KDL::JntArray lleg_joint_position;
  KDL::Frame rleg_current_pose;
  KDL::Frame lleg_current_pose;
  KDL::Frame pelvis_current_pose;
  KDL::Frame pelvis_desired_pose;


//Joint limits
//Right leg
  KDL::JntArray rleg_min_joint_limit;
  KDL::JntArray rleg_max_joint_limit;

//Right leg
  KDL::Chain rleg_pelvis_to_foot_chain;
  KDL::Chain rleg_foot_to_pelvis_chain;
//Pelvis->foot solvers for leg swing
  KDL::ChainIkSolverPos_LMA *rleg_pelvis_to_foot_ik_pos_solver;
//Foot->pelvis solvers for COM translation
  //KDL::ChainIkSolverPos_LMA *rleg_foot_to_pelvis_ik_pos_solver;

  KDL::ChainFkSolverPos_recursive *rleg_foot_to_pelvis_fk_solver;
  KDL::ChainIkSolverVel_pinv      *rleg_foot_to_pelvis_ik_vel_solver;
  KDL::ChainIkSolverPos_NR_JL     *rleg_foot_to_pelvis_ik_pos_solver;

//Left leg
  KDL::Chain lleg_pelvis_to_foot_chain;
  KDL::Chain lleg_foot_to_pelvis_chain;
//Pelvis->foot solvers for leg swing
  KDL::ChainIkSolverPos_LMA *lleg_pelvis_to_foot_ik_pos_solver;
//Foot->pelvis solvers for COM translation
  KDL::ChainIkSolverPos_LMA *lleg_foot_to_pelvis_ik_pos_solver;

//IK
  Eigen::VectorXd rleg_joint_position_, lleg_joint_position_;

  KDL::JntArray rleg_des_joint_pos;
  KDL::JntArray lleg_des_joint_pos;

//ROS joint Publishers
//Right leg
  ros::Publisher r_hip_y_pub;
  ros::Publisher r_hip_r_pub;
  ros::Publisher r_hip_p_pub;
  ros::Publisher r_kn_p_pub;
  ros::Publisher r_an_p_pub;
  ros::Publisher r_an_r_pub;
//Left leg
  ros::Publisher l_hip_y_pub;
  ros::Publisher l_hip_r_pub;
  ros::Publisher l_hip_p_pub;
  ros::Publisher l_kn_p_pub;
  ros::Publisher l_an_p_pub;
  ros::Publisher l_an_r_pub;

//Right leg
  std_msgs::Float64 r_an_r_msg;
  std_msgs::Float64 r_an_p_msg;
  std_msgs::Float64 r_kn_p_msg;
  std_msgs::Float64 r_hip_p_msg;
  std_msgs::Float64 r_hip_r_msg;
  std_msgs::Float64 r_hip_y_msg;
//Left leg
  std_msgs::Float64 l_an_r_msg;
  std_msgs::Float64 l_an_p_msg;
  std_msgs::Float64 l_kn_p_msg;
  std_msgs::Float64 l_hip_p_msg;
  std_msgs::Float64 l_hip_r_msg;
  std_msgs::Float64 l_hip_y_msg;

//cur_leg
  bool cur_leg_is_right;

};

#endif // OP3_ZMP_LOCOMOTION_H
