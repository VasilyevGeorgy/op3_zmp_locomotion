#include "op3_zmp_quasistatic.h"

int main (int argc, char **argv){

  ros::init(argc,argv,"set_initial_pose");

  op3_zmp_quasistatic move;

  move.launchManager();

  KDL::Frame goalPose = KDL::Frame(KDL::Rotation::RPY(0.0,0.0,0.0),
                                   KDL::Vector(0.0, 0.0, 0.3));

  //move.InitPoseTest(goalPose);

  op3_zmp_quasistatic::stepParam sp;
  sp.freq = 100.0; // HZ
  sp.num_of_steps = 5.0;
  sp.init_leg = "RighT";
  sp.step_length = 30.0;   //mm
  sp.step_duration = 3.0;  //sec
  sp.step_clearance = 15.0; //mm

  //move.quasiStaticPlaner(goalPose, sp);
  //move.locomotion(sp);

  Eigen::VectorXd rleg_joint_pos_;
  Eigen::VectorXd lleg_joint_pos_;
  rleg_joint_pos_.resize(JOINT_NUM);
  lleg_joint_pos_.resize(JOINT_NUM);

  move.goToInitialPose(goalPose, sp);
  move.locomotion(sp);


  //move.managerJointPos();
  //move.getJointPosition(rleg_joint_pos_,lleg_joint_pos_);

  //ROS_INFO("Right leg (deg) hip_yaw:%f, hip_r:%f, hip_p:%f, kn_p:%f, an_p :%f, an_r:%f",
  //         rleg_joint_pos_(0)*R2D,rleg_joint_pos_(1)*R2D,rleg_joint_pos_(2)*R2D,
  //         rleg_joint_pos_(3)*R2D,rleg_joint_pos_(4)*R2D,rleg_joint_pos_(5)*R2D);
  //
  //ROS_INFO("Left leg (deg) hip_yaw:%f, hip_r:%f, hip_p:%f, kn_p:%f, an_p :%f, an_r:%f",
  //         lleg_joint_pos_(0)*R2D,lleg_joint_pos_(1)*R2D,lleg_joint_pos_(2)*R2D,
  //         lleg_joint_pos_(3)*R2D,lleg_joint_pos_(4)*R2D,lleg_joint_pos_(5)*R2D);



  return 0;
}
