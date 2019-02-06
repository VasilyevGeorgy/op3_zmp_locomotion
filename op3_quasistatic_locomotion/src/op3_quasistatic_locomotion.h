#ifndef OP3_QUASISTATIC_LOCOMOTION_H
#define OP3_QUASISTATIC_LOCOMOTION_H

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <cmath>
#include <ctime>
#include <vector>
#include <map>
#include <limits>

//#include <ctime>
//#include "ctime"

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/GetJointModule.h"
#include "robotis_controller_msgs/SetModule.h"
#include "robotis_controller_msgs/SetJointModule.h"

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

class op3_quasistatic_locomotion
{
public:
  op3_quasistatic_locomotion();
  virtual ~op3_quasistatic_locomotion();

  struct stepParam{
    stepParam(double freq = 200, double num_of_steps = 1.0,
              std::string init_leg = "left", double step_length = 30.0,
              double step_duration = 3.0, double step_clearance = 15.0)
              : freq(freq), num_of_steps(num_of_steps), init_leg(init_leg), step_length(step_length), step_duration(step_duration), step_clearance(step_clearance)
    {}
    double freq;
    double num_of_steps;
    std::string init_leg;
    double step_length; //mm
    double step_duration; //sec
    double step_clearance; //mm
  };

  bool launchManager();
  void InitPoseTest(KDL::Frame pelvis_des_pose);
  void goToInitialPose(KDL::Frame pelvis_des_pose, stepParam sp);
  void quasiStaticPlaner(KDL::Frame pelvis_des_pose, stepParam sp);
  void locomotion(stepParam sp);
  void perFrameLocomotion(unsigned long int num_of_frame);
  void getAnglesVectors(std::vector<Eigen::VectorXd> &rleg_joint_angles_,
                        std::vector<Eigen::VectorXd> &lleg_joint_angles_);
  void managerJointPos();
  void getJointPosition(Eigen::VectorXd &rleg_joint_position_,
                        Eigen::VectorXd &lleg_joint_position_);


private:
  bool rostopic_is_init;
  //Trajectory and Joint angles' vectors
  //std::vector<KDL::Frame> rfoot_poses;
  //std::vector<KDL::Frame> rfoot_poses;
  //std::vector<KDL::Frame> pelivis_poses;
  std::vector<Eigen::VectorXd> rleg_joint_angles;
  std::vector<Eigen::VectorXd> lleg_joint_angles;
  bool keyboard_run;
  bool keyboard_quit;


  //Step parameters
  //double freq;
  double up_part;
  double down_part;

  //Pose
  KDL::Frame pelvis_pose;
  KDL::Frame rfoot_pose;
  KDL::Frame lfoot_pose;

  //Joint arrays
  KDL::JntArray rleg_joint_pos;
  KDL::JntArray lleg_joint_pos;

  ////IK
  //KDL::JntArray rleg_des_joint_pos;
  //KDL::JntArray lleg_des_joint_pos;

  //Joint limits
  KDL::JntArray rleg_min_pos_limit;
  KDL::JntArray rleg_max_pos_limit;
  KDL::JntArray lleg_min_pos_limit;
  KDL::JntArray lleg_max_pos_limit;

  //Kinematic chains and solvers
  KDL::Chain *rleg_chain;
  KDL::Chain *lleg_chain;
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
  void presStateCallback(const sensor_msgs::JointState::ConstPtr &jnt_data);
  //void get_joints_pos();
  bool getJointCallback;
  bool manager_is_launched;


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

  ros::Publisher leg_joints_pub;

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

  void deleteChains();
  void deleteSolvers();

  void initialization(KDL::Frame pelvis_pose);
  void initializeChains(KDL::Frame pelvis_pose);
  void initializeROS();

  void setJointPosition(Eigen::VectorXd rleg_joint_position_, Eigen::VectorXd lleg_joint_position_);
  bool getFeetPose();
  bool moveFoot(KDL::Frame foot_des_pose, Eigen::VectorXd &leg_des_joint_pos_, std::string legType);
  bool movePelvis(KDL::Frame leg_des_pose, Eigen::VectorXd &leg_des_joint_pos_, std::string legType);
  bool footTrajectoryGeneration(std::vector<KDL::Frame> &foot_poses, stepParam sp, std::string legType);
  void initCoMTranslation(std::string legType, stepParam sp);
  void footTranslation(stepParam sp, std::string legType);
  void translateCoM(std::string legType, stepParam sp);
  void publishMessageROS(Eigen::VectorXd rleg_jnt_angle_, Eigen::VectorXd lleg_jnt_angle_);
  void keyboardContolCallback(const std_msgs::String::ConstPtr &cntrl_status);
  bool getCurrentModule(std::vector<std::string> &joint_name, std::vector<std::string> &joint_module);
  void setJointModule(const std::vector<std::string> &joint_name, const std::vector<std::string> &module_name); // service call to set module
  void setLegsModule(std::string moduleName); // Module for legs joints
  void setModule(std::string moduleName); // Module for whole joints



};

#endif // OP3_QUASISTATIC_LOCOMOTION_H
