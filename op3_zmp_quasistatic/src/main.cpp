#include "op3_zmp_quasistatic.h"

//void thrGetInput(){
//  bool stop = false;
//  char input_char;
//
//  clock_t start = std::clock();
//  clock_t cur_time = std::clock();
//
//  while((cur_time=clock()-start)/CLOCKS_PER_SEC != 0.005){
//
//  }
//
//}

//while((c=std::getchar())!= 27) {


int main (int argc, char **argv){

  ros::init(argc,argv,"set_initial_pose");

  op3_zmp_quasistatic move;

  //move.launchManager();

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

  move.quasiStaticPlaner(goalPose, sp);
  move.locomotion(sp);

  //std::vector<Eigen::VectorXd> rleg_joint_angles_;
  //std::vector<Eigen::VectorXd> lleg_joint_angles_;
  //
  //move.getAnglesVectors(rleg_joint_angles_, lleg_joint_angles_);
  //
  //ROS_INFO("RIGHT LEG angles vector size: %lu", rleg_joint_angles_.size());
  //ROS_INFO(" LEFT LEG angles vector size: %lu", lleg_joint_angles_.size());

  //ros::Rate rate (sp.freq);
  //for(unsigned long int i=0;i<rleg_joint_angles_.size();i++){
  //  move.perFrameLocomotion(i);
  //  rate.sleep();
  //}

  //unsigned long int counter = 0;
  //
  //system ("/bin/stty raw");
  //
  //while((ros::ok())&&(counter<rleg_joint_angles_.size())){
  //
  //  std::thread thr_key_input(thrGetInput);
  //  thr_key_input.join();
  //
  //  move.perFrameLocomotion(counter);
  //  counter++;
  //
  //}
  //
  //// use system call to set terminal behaviour to more normal behaviour
  //system ("/bin/stty cooked");



  return 0;
}
