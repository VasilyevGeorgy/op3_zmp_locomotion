#include "ros/ros.h"
#include "kdl_3dof_planar_man/IKServer.h"
#include "kdl_3dof_planar_man/FKServer.h"
#include "3dof_planar_man.h"

bool IK(kdl_3dof_planar_man::IKServer::Request &ik_req,
          kdl_3dof_planar_man::IKServer::Response &ik_res){

  Planar_man manipulator;
  std::vector<double_t> angles = manipulator.solveIK(Eigen::Vector3d(0.0,ik_req.x,ik_req.y),
                                                        Eigen::Vector3d(ik_req.roll,0.0,0.0));

  ik_res.th1 = angles[0]*R2D;
  ik_res.th2 = angles[1]*R2D;
  ik_res.th3 = angles[2]*R2D;

  manipulator.setInitialPosition(30.0,30.0,30.0);

  return true;

}


bool FK(kdl_3dof_planar_man::FKServer::Request &fk_req,
          kdl_3dof_planar_man::FKServer::Response &fk_res){

  Planar_man manipulator;
  std::vector<double_t> ik_out = manipulator.solveFK();

  fk_res.x = ik_out[0];
  fk_res.y = ik_out[1];
  fk_res.roll = ik_out[2]*R2D;

  return true;

}


int main(int argc,char**argv){

  ros::init(argc,argv,"add_two_ints_server");
  ros::NodeHandle nh;

  //Planar_man *manipulator;
  //manipulator = new Planar_man();

  ros::ServiceServer service_IK = nh.advertiseService("solveIK",IK);
  ros::ServiceServer service_FK = nh.advertiseService("solveFK",FK);
  ROS_INFO("IK_server is run!");

  ros::spin();

  //delete manipulator;

  return 0;
}
