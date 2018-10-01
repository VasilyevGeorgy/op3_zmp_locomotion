#ifndef PLANAR_MAN_SERVICES_H
#define PLANAR_MAN_SERVICES_H

#include "3dof_planar_man.h"
#include "kdl_3dof_planar_man/IKServer.h"
#include "kdl_3dof_planar_man/FKServer.h"

class Planar_man_services {

public:
  Planar_man_services();
  virtual ~Planar_man_services();

  void runServices();

private:
  ros::NodeHandle nh;
  Planar_man manipulator;
  

};


#endif
