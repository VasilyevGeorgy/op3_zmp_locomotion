#include "op3_zmp_inv_kin.h"


int main (int argc, char **argv){

  ros::init(argc,argv,"set_initial_pose");

  op3_zmp_inv_kin move;

  ROS_INFO("Test timer 0");

  std::system(RUNMNGRSCRIPT);

  ros::Duration(5.0).sleep();

  ROS_INFO("Test timer 1");

  KDL::Frame goalPose = KDL::Frame(KDL::Rotation::RPY(0.0,0.0,0.0),
                                   KDL::Vector(0.0,0.0,0.25));

  move.goToInitialPose(goalPose);

  return 0;
}
