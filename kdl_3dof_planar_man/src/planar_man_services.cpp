#include "planar_man_services.h"

Planar_man_services::Planar_man_services()
{
  manipulator = new Planar_man();

}
Planar_man_services::~Planar_man_services(){
  delete manipulator;
}

bool Planar_man_services::SetInit(kdl_3dof_planar_man::SetInitServer::Request &init_req,
          kdl_3dof_planar_man::SetInitServer::Response &init_res){

   manipulator->setInitialPosition(init_req.th1, init_req.th2, init_req.th3);

  return true;

}

bool Planar_man_services::IK(kdl_3dof_planar_man::IKServer::Request &ik_req,
                        kdl_3dof_planar_man::IKServer::Response &ik_res)
{
  std::vector<double_t> angles = manipulator->solveIK(Eigen::Vector3d(0.0,ik_req.x,ik_req.y),
                                                        Eigen::Vector3d(ik_req.roll,0.0,0.0));

  ik_res.th1 = angles[0]*R2D;
  ik_res.th2 = angles[1]*R2D;
  ik_res.th3 = angles[2]*R2D;

  return true;

}

bool Planar_man_services::FK(kdl_3dof_planar_man::FKServer::Request &fk_req,
          kdl_3dof_planar_man::FKServer::Response &fk_res){

  std::vector<double_t> ik_out = manipulator->solveFK(Eigen::Vector3d(fk_req.th1,
                                                                      fk_req.th2,
                                                                      fk_req.th3));

  fk_res.x = ik_out[0];
  fk_res.y = ik_out[1];
  fk_res.roll = ik_out[2]*R2D;

  return true;

}

bool Planar_man_services::FK_empty(kdl_3dof_planar_man::FK_empty::Request &fk_req,
          kdl_3dof_planar_man::FK_empty::Response &fk_res){

  std::vector<double_t> ik_out = manipulator->solveFK();

  fk_res.x = ik_out[0];
  fk_res.y = ik_out[1];
  fk_res.roll = ik_out[2]*R2D;

  return true;

}

void Planar_man_services::runServices(){

  //Planar_man_services service;
  ros::ServiceServer service_SetInit = nh.advertiseService("set_initial_pose",
                                                        &Planar_man_services::SetInit,this);
  ros::ServiceServer service_IK = nh.advertiseService("get_joint_postiion",
                                                        &Planar_man_services::IK,this);
  ros::ServiceServer service_FK = nh.advertiseService("get_tip_pose",
                                                        &Planar_man_services::FK,this);
  ros::ServiceServer service_FK_empty = nh.advertiseService("get_current_tip_pose",
                                                        &Planar_man_services::FK_empty,this);

  ROS_INFO("Services are run!");

  ros::spin();

}
