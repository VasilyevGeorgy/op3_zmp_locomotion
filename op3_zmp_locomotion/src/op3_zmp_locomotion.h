#ifndef OP3_ZMP_LOCOMOTION_H
#define OP3_ZMP_LOCOMOTION_H

#include <string>
#include <cmath>
#include <stdint.h>
#include <vector>
#include <map>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include "std_msgs/Float64.h"

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
  void move_pelvis(Eigen::VectorXd &rleg_des_joint_pos_, Eigen::VectorXd &lleg_des_joint_pos_);

  bool moveLeftLeg(KDL::Frame lleg_des_pose, Eigen::VectorXd &lleg_des_joint_pos_);
  bool moveCOMToLeftLeg(KDL::Frame pelvis_des_pose, Eigen::VectorXd &lleg_des_joint_pos_);

  void comTranslation(Eigen::VectorXd &rleg_cur_joint_pos_, Eigen::VectorXd &lleg_cur_joint_pos_);

  void legSwing(Eigen::VectorXd &rleg_cur_joint_pos_, Eigen::VectorXd &lleg_cur_joint_pos_);

  bool moveRightLeg(KDL::Frame rleg_des_pose, Eigen::VectorXd &rleg_des_joint_pos_);
  bool moveCOMToRightLeg(KDL::Frame pelvis_des_pose, Eigen::VectorXd &rleg_des_joint_pos_);

  void quit(); //delete chains

  KDL::JntArray rleg_joint_position;
  KDL::JntArray lleg_joint_position;
  KDL::Frame rleg_current_pose;
  KDL::Frame lleg_current_pose;
  KDL::Frame rleg_desired_pose;
  KDL::Frame lleg_desired_pose;
  KDL::Frame pelvis_current_pose;
  KDL::Frame pelvis_desired_pose;

  //Joint limits
  //Right leg
    KDL::JntArray rleg_foot_to_pelvis_min_position_limit;
    KDL::JntArray rleg_foot_to_pelvis_max_position_limit;
    KDL::JntArray rleg_pelvis_to_foot_min_position_limit;
    KDL::JntArray rleg_pelvis_to_foot_max_position_limit;
  //Left leg
    KDL::JntArray lleg_foot_to_pelvis_min_position_limit;
    KDL::JntArray lleg_foot_to_pelvis_max_position_limit;
    KDL::JntArray lleg_pelvis_to_foot_min_position_limit;
    KDL::JntArray lleg_pelvis_to_foot_max_position_limit;

  //Right leg chains and solvers
    KDL::Chain rleg_pelvis_to_foot_chain;
    KDL::Chain rleg_foot_to_pelvis_chain;
  //Pelvis->foot solvers for leg swing
    KDL::ChainFkSolverPos_recursive *rleg_pelvis_to_foot_fk_solver;
    KDL::ChainIkSolverVel_pinv      *rleg_pelvis_to_foot_ik_vel_solver;
    KDL::ChainIkSolverPos_NR_JL     *rleg_pelvis_to_foot_ik_pos_solver;
  //Foot->pelvis solvers for COM translation
    KDL::ChainFkSolverPos_recursive *rleg_foot_to_pelvis_fk_solver;
    KDL::ChainIkSolverVel_pinv      *rleg_foot_to_pelvis_ik_vel_solver;
    KDL::ChainIkSolverPos_NR_JL     *rleg_foot_to_pelvis_ik_pos_solver;


  //Left leg chains and solvers
    KDL::Chain lleg_pelvis_to_foot_chain;
    KDL::Chain lleg_foot_to_pelvis_chain;
  //Pelvis->foot solvers for leg swing
    KDL::ChainFkSolverPos_recursive *lleg_pelvis_to_foot_fk_solver;
    KDL::ChainIkSolverVel_pinv      *lleg_pelvis_to_foot_ik_vel_solver;
    KDL::ChainIkSolverPos_NR_JL     *lleg_pelvis_to_foot_ik_pos_solver;
    //KDL::ChainIkSolverPos_LMA *lleg_pelvis_to_foot_ik_pos_solver;
  //Foot->pelvis solvers for COM translation
    KDL::ChainFkSolverPos_recursive *lleg_foot_to_pelvis_fk_solver;
    KDL::ChainIkSolverVel_pinv      *lleg_foot_to_pelvis_ik_vel_solver;
    KDL::ChainIkSolverPos_NR_JL     *lleg_foot_to_pelvis_ik_pos_solver;

  //IK
    Eigen::VectorXd rleg_joint_position_, lleg_joint_position_;

    KDL::JntArray rleg_des_joint_pos;
    KDL::JntArray lleg_des_joint_pos;

    bool init_com_transl;


private:
  ros::NodeHandle node;



  double cur_r_hip_y;
  double cur_r_hip_r;
  double cur_r_hip_p;
  double cur_r_kn_p;
  double cur_r_an_p;
  double cur_r_an_r;

  void r_hip_y_callback(const std_msgs::Float64::ConstPtr &angle);
  void r_hip_r_callback(const std_msgs::Float64::ConstPtr &angle);
  void r_hip_p_callback(const std_msgs::Float64::ConstPtr &angle);
  void r_kn_p_callback(const std_msgs::Float64::ConstPtr &angle);
  void r_an_p_callback(const std_msgs::Float64::ConstPtr &angle);
  void r_an_r_callback(const std_msgs::Float64::ConstPtr &angle);



//cur_leg
  bool cur_leg_is_right;

//ROS joint Subscribers
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


};

#endif // OP3_ZMP_LOCOMOTION_H
