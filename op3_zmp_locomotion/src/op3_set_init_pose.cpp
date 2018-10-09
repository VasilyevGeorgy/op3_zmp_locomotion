#include "op3_zmp_locomotion.h"

#define PELVIS_INIT_POSITION (0.3697)

void op3_zmp_locomotion::move_pelvis(KDL::Frame &pelvis_des_pose, double &pelvis_current_position,
                                      Eigen::VectorXd &rleg_des_joint_pos_, Eigen::VectorXd &lleg_des_joint_pos_){

  this->moveCOMToRightLeg(KDL::Frame(pelvis_des_pose.M,
                          //KDL::Frame(KDL::Rotation::RPY(pelvis_roll, pelvis_pitch, pelvis_yaw),
                                  KDL::Vector(0.0, 0.0, pelvis_current_position)),
                                  rleg_des_joint_pos_);

  this->moveCOMToLeftLeg(KDL::Frame(pelvis_des_pose.M,
                         //KDL::Frame(KDL::Rotation::RPY(pelvis_roll, pelvis_pitch, pelvis_yaw),
                                  KDL::Vector(0.0, 0.0, pelvis_current_position)),
                                  lleg_des_joint_pos_);

//Right leg
  r_an_r_msg.data = rleg_des_joint_pos_(0);
  r_an_p_msg.data = -rleg_des_joint_pos_(1);
  r_kn_p_msg.data = -rleg_des_joint_pos_(2);
  r_hip_p_msg.data = -rleg_des_joint_pos_(3);
  r_hip_r_msg.data = rleg_des_joint_pos_(4);
  r_hip_y_msg.data = rleg_des_joint_pos_(5);
//Left leg
  l_an_r_msg.data = lleg_des_joint_pos_(0);
  l_an_p_msg.data = lleg_des_joint_pos_(1);
  l_kn_p_msg.data = lleg_des_joint_pos_(2);
  l_hip_p_msg.data = lleg_des_joint_pos_(3);
  l_hip_r_msg.data = lleg_des_joint_pos_(4);
  l_hip_y_msg.data = lleg_des_joint_pos_(5);
//From pelvis to ankles
  r_hip_y_pub.publish(r_hip_y_msg);
  l_hip_y_pub.publish(l_hip_y_msg);

  r_hip_r_pub.publish(r_hip_r_msg);
  l_hip_r_pub.publish(l_hip_r_msg);

  r_hip_p_pub.publish(r_hip_p_msg);
  l_hip_p_pub.publish(l_hip_p_msg);

  r_kn_p_pub.publish(r_kn_p_msg);
  l_kn_p_pub.publish(l_kn_p_msg);

  r_an_p_pub.publish(r_an_p_msg);
  l_an_p_pub.publish(l_an_p_msg);

  r_an_r_pub.publish(r_an_r_msg);
  l_an_r_pub.publish(l_an_r_msg);

}

bool op3_zmp_locomotion::setInitPose(KDL::Frame pelvis_des_pose, Eigen::VectorXd &rleg_cur_joint_pos_, Eigen::VectorXd &lleg_cur_joint_pos_){

  ros::Rate loop_rate(30);

  //op3_zmp_locomotion move_pelvis;

  rleg_current_pose.p.data[0] = 0.0;
  rleg_current_pose.p.data[1] = -0.035;

  lleg_current_pose.p.data[0] = 0.0;
  lleg_current_pose.p.data[1] = 0.035;

  this->initialize(pelvis_des_pose,
                    KDL::Frame(KDL::Vector(0.0 , -0.035, 0.0)),
                    KDL::Frame(KDL::Vector(0.0 , 0.035, 0.0)));

  double pelvis_roll;
  double pelvis_pitch;
  double pelvis_yaw;

  pelvis_des_pose.M.GetRPY(pelvis_roll,pelvis_pitch,pelvis_yaw);

  Eigen::VectorXd rleg_joint_pos_, lleg_joint_pos_;

  rleg_joint_pos_.resize(JOINT_NUM);
  rleg_joint_pos_(0) = 0.001;
  rleg_joint_pos_(1) = 0.001;
  rleg_joint_pos_(2) = 0.001;
  rleg_joint_pos_(3) = 0.001;
  rleg_joint_pos_(4) = 0.001;
  rleg_joint_pos_(5) = 0.001;

  lleg_joint_pos_.resize(JOINT_NUM);
  lleg_joint_pos_(0) = 0.001;
  lleg_joint_pos_(1) = 0.001;
  lleg_joint_pos_(2) = 0.001;
  lleg_joint_pos_(3) = 0.001;
  lleg_joint_pos_(4) = 0.001;
  lleg_joint_pos_(5) = 0.001;

  this->setJointPosition(rleg_joint_pos_,lleg_joint_pos_);

  Eigen::VectorXd rleg_des_joint_pos_;
  Eigen::VectorXd lleg_des_joint_pos_;

  rleg_des_joint_pos_.resize(JOINT_NUM);
  lleg_des_joint_pos_.resize(JOINT_NUM);

  //r_hip_y_sub = node.subscribe("robotis_op3/r_hip_yaw_position/command",1, &op3_zmp_locomotion::r_hip_y_callback, this);
  //r_hip_y_sub.shutdown();


  //FK
  KDL::JntArray rleg_joint_pos;
  KDL::JntArray lleg_joint_pos;

  rleg_joint_pos.resize(JOINT_NUM);
  lleg_joint_pos.resize(JOINT_NUM);

  rleg_joint_pos(0) = cur_r_hip_y;
  rleg_joint_pos(1) = cur_r_hip_r;
  rleg_joint_pos(2) = cur_r_hip_p;
  rleg_joint_pos(3) = cur_r_kn_p;
  rleg_joint_pos(4) = cur_r_an_p;
  rleg_joint_pos(5) = cur_r_an_r;

  lleg_joint_pos(0) = cur_r_hip_y;
  lleg_joint_pos(1) = cur_r_hip_r;
  lleg_joint_pos(2) = cur_r_hip_p;
  lleg_joint_pos(3) = cur_r_kn_p;
  lleg_joint_pos(4) = cur_r_an_p;
  lleg_joint_pos(5) = cur_r_an_r;

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  double pelvis_current_position = PELVIS_INIT_POSITION;

  int fk_pose_err = rleg_foot_to_pelvis_fk_solver->JntToCart(rleg_joint_pos, pelvis_current_pose);

  //if (fk_pose_err == 0)
  //pelvis_current_position = pelvis_current_pose.p.z();

  //ROS_INFO("\n\n\n%f\n\n\n", pelvis_current_pose.p.z());


//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


  double pelvis_target_position_z = pelvis_des_pose.p.z();
  double step = 0.0002;

  ROS_INFO("SetInitPose is run!");

  while (pelvis_current_position > pelvis_target_position_z){


    pelvis_current_position -= step;

    pelvis_current_pose = KDL::Frame(KDL::Rotation::RPY(pelvis_roll, pelvis_pitch, pelvis_yaw), KDL::Vector(0.0, 0.0, pelvis_current_position));

    move_pelvis(pelvis_des_pose, pelvis_current_position, rleg_des_joint_pos_, lleg_des_joint_pos_);

    ros::spinOnce();
    loop_rate.sleep();

  }

  while (pelvis_current_position < pelvis_target_position_z){

    pelvis_current_position += step;

    pelvis_current_pose = KDL::Frame(KDL::Rotation::RPY(pelvis_roll, pelvis_pitch, pelvis_yaw), KDL::Vector(0.0, 0.0, pelvis_current_position));

    move_pelvis(pelvis_des_pose, pelvis_current_position, rleg_des_joint_pos_, lleg_des_joint_pos_);

    ros::spinOnce();
    loop_rate.sleep();

  }

  ROS_INFO("Initial pose is reached: %f", pelvis_current_pose.p.z());

  rleg_cur_joint_pos_ = rleg_des_joint_pos_;
  lleg_cur_joint_pos_ = lleg_des_joint_pos_;
  //pelvis_cur_pose = pelvis_current_pose;

  return true;

}
