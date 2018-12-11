#include "op3_zmp_quasistatic.h"

int main (int argc, char **argv){

  ros::init(argc,argv,"set_initial_pose");

  op3_zmp_quasistatic move;

  //move.launchManager();

  KDL::Frame goalPose = KDL::Frame(KDL::Rotation::RPY(0.0,0.0,0.0),
                                   KDL::Vector(0.0, 0.0, 0.3));

  //move.InitPoseTest(goalPose);

  op3_zmp_quasistatic::stepParam sp;
  sp.num_of_steps = 5.0;
  sp.init_leg = "Left";
  sp.step_length = 30.0;   //mm
  sp.step_duration = 3.0;  //sec
  sp.step_clearance = 15.0; //mm

  //move.goToInitialPose(goalPose);
  move.quasiStaticPlaner(goalPose, sp);
  move.locomotion();

  return 0;
}
