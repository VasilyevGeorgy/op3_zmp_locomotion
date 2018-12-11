#include "op3_zmp_quasistatic_service.h"

op3_zmp_quasistaic_service::op3_zmp_quasistaic_service()
{
  //TODO: params input while thru server launch
  this->pelvis_init_pose = KDL::Frame(KDL::Rotation::RPY(0.0,0.0,0.0),
                                          KDL::Vector(0.0, 0.0, 0.3));

  this->step_params.init_leg = "Left";
  this->step_params.step_length = 30.0;   //mm
  this->step_params.step_duration = 3.0;  //sec
  this->step_params.step_clearance = 15.0; //mm

  start_running = false;

}

op3_zmp_quasistaic_service::~op3_zmp_quasistaic_service(){

}

bool op3_zmp_quasistaic_service::run_server(op3_zmp_quasistatic::op3_zmp_quasistatic_serviceRequest &req,
                                            op3_zmp_quasistatic::op3_zmp_quasistatic_serviceResponse &res)
{
  //std::transform(legType.begin(),legType.end(),legType.begin(), ::tolower);


  if (req == "start"){
    start_running = true;
    //system("/bin/stty raw");
  }



  //if (req == "stop"){
  //
  //}
  //if (req == "go"){
  //
  //}



}

void op3_zmp_quasistaic_service::launch_service(){

  this->robot.quasiStaticPlaner(pelvis_init_pose, step_params);
  this->counter = this->robot.rleg_joint_angles.size();

  //int c;
  ///* use system call to make terminal send all keystrokes directly to stdin */
  //system ("/bin/stty raw");
  //while((c=std::getchar())!= 27) {
  //
  //  /* type a period to break out of the loop, since CTRL-D won't work raw */
  //  //std::putchar(c);
  //}
  ///* use system call to set terminal behaviour to more normal behaviour */
  //system ("/bin/stty cooked");

}


