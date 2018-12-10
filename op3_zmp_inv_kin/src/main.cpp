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
  sp.init_leg = "Left";
  sp.step_length = 30.0;   //mm
  sp.step_duration = 3.0;  //sec
  sp.step_clearance = 15.0; //mm

  //move.goToInitialPose(goalPose);
  move.quasiStatic(goalPose, sp, 10);

  return 0;
}
