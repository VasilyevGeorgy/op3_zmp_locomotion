#ifndef PLANAR_MAN_SERVICES_H
#define PLANAR_MAN_SERVICES_H

#include "3dof_planar_man.h"
#include "kdl_3dof_planar_man/IKServer.h"
#include "kdl_3dof_planar_man/FKServer.h"
#include "kdl_3dof_planar_man/SetInitServer.h"
#include "kdl_3dof_planar_man/FK_empty.h"

class Planar_man_services
{
public:
  Planar_man_services();
  virtual ~Planar_man_services();

  void runServices();

private:
  ros::NodeHandle nh;
  Planar_man *manipulator;

  bool SetInit(kdl_3dof_planar_man::SetInitServer::Request &,
            kdl_3dof_planar_man::SetInitServer::Response &);

  bool IK(kdl_3dof_planar_man::IKServer::Request &,
           kdl_3dof_planar_man::IKServer::Response &);

  bool FK(kdl_3dof_planar_man::FKServer::Request &,
           kdl_3dof_planar_man::FKServer::Response &);

  bool FK_empty(kdl_3dof_planar_man::FK_empty::Request &,
           kdl_3dof_planar_man::FK_empty::Response &);

};

#endif // PLANAR_MAN_SERVICES_H
