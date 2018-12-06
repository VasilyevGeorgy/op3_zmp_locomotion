#ifndef OP3_ZMP_INV_KIN_H
#define OP3_ZMP_INV_KIN_H

#include <stdlib.h>
#include <string>
#include <cmath>
#include <stdint.h>
#include <vector>
#include <map>
#include <limits>


#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"

#include <eigen3/Eigen/Eigen>

#include <kdl/joint.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainiksolverpos_lma.hpp>

#define RUNMNGRSCRIPT "\
#!/bin/bash \n\
gnome-terminal -e \"roslaunch op3_manager op3_gazebo.launch\" \
"

//<node name="foo" pkg="bar" type="bar_node" output="screen" launch-prefix="xterm -e" />
//roslaunch op3_manager op3_gazebo.launch\
//gnome-terminal -e "roslaunch op3_gazebo robotis_world.launch"

const int JOINT_NUM = 6;
const double D2R = M_PI/180.0;
const double R2D = 180.0/M_PI;
const double y_offset = 0.035;

//#define JOINT_NUM (6)
//#define D2R (M_PI/180.0)
//#define R2D (180.0/M_PI)

class op3_zmp_inv_kin
{
public:
  op3_zmp_inv_kin();
  virtual ~op3_zmp_inv_kin();

  void goToInitialPose(KDL::Frame pelvis_des_pose);

private:
  //Pose
  KDL::Frame pelvis_pose;
  KDL::Frame rfoot_pose;
  KDL::Frame lfoot_pose;

  //Joint arrays
  KDL::JntArray rleg_joint_pos;
  KDL::JntArray lleg_joint_pos;

  Eigen::VectorXd all_joints; //!!!!!

  //IK
  KDL::JntArray rleg_des_joint_pos;
  KDL::JntArray lleg_des_joint_pos;

  //Joint limits
  KDL::JntArray rleg_min_pos_limit;
  KDL::JntArray rleg_max_pos_limit;
  KDL::JntArray lleg_min_pos_limit;
  KDL::JntArray lleg_max_pos_limit;

  //Kinematic chains and solvers
  KDL::Chain rleg_chain;
  KDL::Chain lleg_chain;
  //Right leg
  KDL::ChainFkSolverPos_recursive *rleg_fk_solver;
  KDL::ChainIkSolverVel_pinv      *rleg_ik_vel_solver;
  KDL::ChainIkSolverPos_NR_JL     *rleg_ik_pos_solver;
  //Left leg
  KDL::ChainFkSolverPos_recursive *lleg_fk_solver;
  KDL::ChainIkSolverVel_pinv      *lleg_ik_vel_solver;
  KDL::ChainIkSolverPos_NR_JL     *lleg_ik_pos_solver;

  //ROS
  ros::NodeHandle node;
  //Joint Subscribers
  //Right leg
  ros::Subscriber r_hip_y_sub;
  ros::Subscriber r_hip_r_sub;
  ros::Subscriber r_hip_p_sub;
  ros::Subscriber r_kn_p_sub;
  ros::Subscriber r_an_p_sub;
  ros::Subscriber r_an_r_sub;
  //Left leg
  ros::Subscriber l_hip_y_sub;
  ros::Subscriber l_hip_r_sub;
  ros::Subscriber l_hip_p_sub;
  ros::Subscriber l_kn_p_sub;
  ros::Subscriber l_an_p_sub;
  ros::Subscriber l_an_r_sub;

  ros::Subscriber present_joint_states_sub;
  void pres_state_callback(const sensor_msgs::JointState::ConstPtr &jnt_data);
  void get_joints_pos();
  bool getCallback;


  //Joint Publishers
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
  //Messages
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

  void initialization(KDL::Frame pelvis_pose);
  void initializeChains(KDL::Frame pelvis_pose);
  void initializeROS();

  void setJointPosition(Eigen::VectorXd rleg_joint_position_, Eigen::VectorXd lleg_joint_position_);
  bool movePelvis(KDL::Frame leg_des_pose, Eigen::VectorXd &leg_des_joint_pos_, std::string legType);
  void publishMessageROS(Eigen::VectorXd rleg_jnt_angle_, Eigen::VectorXd lleg_jnt_angle_);




};

#endif // OP3_ZMP_INV_KIN_H
