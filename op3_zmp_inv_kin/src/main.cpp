#include "op3_zmp_inv_kin.h"


int main (int argc, char **argv){

  ros::init(argc,argv,"set_initial_pose");

  op3_zmp_inv_kin move;

  ROS_INFO("Test timer 0");

  //move.launchManager();

  ROS_INFO("Test timer 1");

  KDL::Frame goalPose = KDL::Frame(KDL::Rotation::RPY(0.0,0.0,0.0),
                                   KDL::Vector(0.0, 0.0, 0.3));

  //move.InitPoseTest(goalPose);

  op3_zmp_inv_kin::stepParam sp;
  sp.step_length = 10.0;
  sp.step_duration = 8.0;
  sp.step_clearance = 5.0;

  move.goToInitialPose(goalPose, sp);

  return 0;
}
