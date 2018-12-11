#ifndef OP3_ZMP_QUASISTATIC_SERVICE_H
#define OP3_ZMP_QUASISTATIC_SERVICE_H

#include "op3_zmp_quasistatic.h"
#include "op3_zmp_quasistatic/op3_zmp_quasistatic_service.h"

class op3_zmp_quasistaic_service
{
public:
  op3_zmp_quasistaic_service();
  virtual ~op3_zmp_quasistaic_service();

  void launch_service();

private:
  ros::NodeHandle node;

  op3_zmp_quasistatic robot;
  KDL::Frame pelvis_init_pose;
  op3_zmp_quasistatic::stepParam step_params;

  bool start_running;
  unsigned long int counter;

  bool run_server(op3_zmp_quasistatic::op3_zmp_quasistatic_serviceRequest &req,
                  op3_zmp_quasistatic::op3_zmp_quasistatic_serviceResponse &res);

};


#endif // OP3_ZMP_QUASISTATIC_SERVICE_H
