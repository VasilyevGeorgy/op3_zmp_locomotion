#include "op3_kdl_move_pelvis.h"


op3_kdl_move_pelvis::op3_kdl_move_pelvis()
{
  rleg_joint_position.resize(JOINT_NUM);
  lleg_joint_position.resize(JOINT_NUM);

  for (int i=0; i < JOINT_NUM; i++){
    rleg_joint_position(i) = 0.0;
    lleg_joint_position(i) = 0.0;
  }

//Rigth leg chain
// From Foot To Pelvis
  rleg_chain.addSegment(KDL::Segment("base", // Origin
                                     KDL::Joint(KDL::Joint::None),
                                     KDL::Frame(KDL::Vector(0.0 , -0.035, 0.0)), // from base to foot
                                     KDL::RigidBodyInertia(0.0,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
//2
  rleg_chain.addSegment(KDL::Segment("r_leg_end", // !!! probably, it's lower surface of foot
                                     KDL::Joint(KDL::Joint::None),
                                     KDL::Frame(KDL::Vector(0.0 , 0.0 , 0.0305)), // from foot to ankle_roll joit
                                     KDL::RigidBodyInertia(0.1,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );

//3
  rleg_chain.addSegment(KDL::Segment("r_leg_an_r",
                                     KDL::Joint(KDL::Joint::RotX),
                                     KDL::Frame(KDL::Vector(0.0, 0.0, 0.0)),
                                     KDL::RigidBodyInertia(0.06934,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
//4
  rleg_chain.addSegment(KDL::Segment("r_leg_an_p",
                                     KDL::Joint(KDL::Joint::RotY),
                                     KDL::Frame(KDL::Vector(0.0, 0.0, 0.11)),
                                     KDL::RigidBodyInertia(0.17886,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
//5
  rleg_chain.addSegment(KDL::Segment("r_leg_kn_p",
                                     KDL::Joint("minus_RotY", KDL::Vector(0,0,0), KDL::Vector(0,-1,0), KDL::Joint::RotAxis),
                                     KDL::Frame(KDL::Vector(0.0, 0.0, 0.11)),
                                     KDL::RigidBodyInertia(0.04015,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
//6
  rleg_chain.addSegment(KDL::Segment("r_leg_hip_p",
                                     KDL::Joint("minus_RotY", KDL::Vector(0,0,0), KDL::Vector(0,-1,0), KDL::Joint::RotAxis),
                                     KDL::Frame(KDL::Vector(0.0, 0.0, 0.0)),
                                     KDL::RigidBodyInertia(0.11543,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
//7
  rleg_chain.addSegment(KDL::Segment("r_leg_hip_r",
                                     KDL::Joint("minus_RotX", KDL::Vector(0,0,0), KDL::Vector(-1,0,0), KDL::Joint::RotAxis),
                                     KDL::Frame(KDL::Vector(0.0, 0.0, 0.0285)),
                                     KDL::RigidBodyInertia(0.17886,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
//8
  rleg_chain.addSegment(KDL::Segment("r_hip_yaw",
                                     KDL::Joint("minus_RotZ", KDL::Vector(0,0,0), KDL::Vector(0,0,-1), KDL::Joint::RotAxis),
                                     KDL::Frame(KDL::Vector(0.0, 0.035, 0.0907)),
//                                   KDL::Frame(KDL::Vector(0.005, 0.035, 0.0907)),

                                     KDL::RigidBodyInertia(0.01181,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
//9

  rleg_chain.addSegment(KDL::Segment("pelvis", // end of right leg
                                     KDL::Joint(KDL::Joint::None),
                                     KDL::Frame(KDL::Vector(0.0,0.0,0.0)),
                                     KDL::RigidBodyInertia(0.72235,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                    )
                        );


// Left Leg Chain
    lleg_chain.addSegment(KDL::Segment("base", // Origin
                                       KDL::Joint(KDL::Joint::None),
                                       KDL::Frame(KDL::Vector(0.0 , 0.035, 0.0)), // from base to foot
                                       KDL::RigidBodyInertia(0.0,
                                                             KDL::Vector(0.0, 0.0, 0.0),
                                                             KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                             )
                                       )
                          );
//2
    lleg_chain.addSegment(KDL::Segment("l_leg_end", // !!! probably, it's lower surface of foot
                                       KDL::Joint(KDL::Joint::None),
                                       KDL::Frame(KDL::Vector(0.0 , 0.0 , 0.0305)), // from foot to ankle_roll joit
                                       KDL::RigidBodyInertia(0.0,
                                                             KDL::Vector(0.0, 0.0, 0.0),
                                                             KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                             )
                                       )
                          );

//3
    lleg_chain.addSegment(KDL::Segment("l_leg_an_r",
                                       KDL::Joint(KDL::Joint::RotX),
                                       KDL::Frame(KDL::Vector(0.0, 0.0, 0.0)),
                                       KDL::RigidBodyInertia(0.06934,
                                                             KDL::Vector(0.0, 0.0, 0.0),
                                                             KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                             )
                                       )
                          );
//4
    lleg_chain.addSegment(KDL::Segment("l_leg_an_p",
                                       KDL::Joint("minus_RotY", KDL::Vector(0,0,0), KDL::Vector(0,-1,0), KDL::Joint::RotAxis),  ///!!!
                                       KDL::Frame(KDL::Vector(0.0, 0.0, 0.11)),
                                       KDL::RigidBodyInertia(0.17886,
                                                             KDL::Vector(0.0, 0.0, 0.0),
                                                             KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                             )
                                       )
                          );
//5
    lleg_chain.addSegment(KDL::Segment("l_leg_kn_p",
                                       KDL::Joint(KDL::Joint::RotY), ///!!!
                                       KDL::Frame(KDL::Vector(0.0, 0.0, 0.11)),
                                       KDL::RigidBodyInertia(0.04015,
                                                             KDL::Vector(0.0, 0.0, 0.0),
                                                             KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                             )
                                       )
                          );
//6
    lleg_chain.addSegment(KDL::Segment("l_leg_hip_p",
                                       KDL::Joint(KDL::Joint::RotY), ///!!!
                                       KDL::Frame(KDL::Vector(0.0, 0.0, 0.0)),
                                       KDL::RigidBodyInertia(0.11543,
                                                             KDL::Vector(0.0, 0.0, 0.0),
                                                             KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                             )
                                       )
                          );
//7
    lleg_chain.addSegment(KDL::Segment("l_leg_hip_r",
                                       KDL::Joint("minus_RotX", KDL::Vector(0,0,0), KDL::Vector(-1,0,0), KDL::Joint::RotAxis),
                                       KDL::Frame(KDL::Vector(0.0, 0.0, 0.0285)),
                                       KDL::RigidBodyInertia(0.17886,
                                                             KDL::Vector(0.0, 0.0, 0.0),
                                                             KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                             )
                                       )
                          );
//8
    lleg_chain.addSegment(KDL::Segment("l_hip_yaw",
                                       KDL::Joint("minus_RotZ", KDL::Vector(0,0,0), KDL::Vector(0,0,-1), KDL::Joint::RotAxis),
                                       KDL::Frame(KDL::Vector(0.000, -0.035, 0.0907)),
//                                   KDL::Frame(KDL::Vector(0.005, 0.035, 0.0907)),

                                       KDL::RigidBodyInertia(0.01181,
                                                             KDL::Vector(0.0, 0.0, 0.0),
                                                             KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                             )
                                       )
                          );
//9
    lleg_chain.addSegment(KDL::Segment("pelvis", // end of right leg
                                       KDL::Joint(KDL::Joint::None),
                                       KDL::Frame(KDL::Vector(0.0,0.0,0.0)),
                                       KDL::RigidBodyInertia(0.72235,
                                                             KDL::Vector(0.0, 0.0, 0.0),
                                                             KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                             )
                                      )
                          );
// Set Joint Limits
  std::vector<double> min_position_limit_, max_position_limit_;
  min_position_limit_.push_back(-180.0);  max_position_limit_.push_back(180.0); // an_r
  min_position_limit_.push_back(-180.0);	max_position_limit_.push_back(180.0); // an_p
  min_position_limit_.push_back(-180.0);  max_position_limit_.push_back(180.0); // kn_p
  min_position_limit_.push_back(-180.0);	max_position_limit_.push_back(180.0); // hip_p
  min_position_limit_.push_back(-180.0);	max_position_limit_.push_back(180.0); // hip_r
  min_position_limit_.push_back(-180.0);  max_position_limit_.push_back(180.0); // hip_y

  min_joint_position_limit.resize(JOINT_NUM);
  max_joint_position_limit.resize(JOINT_NUM);

  for (int i=0; i<JOINT_NUM; i++)
  {
    min_joint_position_limit(i) = min_position_limit_[i]*D2R; //D2R - degrees to radians
    //ROS_INFO("joint [%d] min: %f", i, min_joint_position_limit(i));
    max_joint_position_limit(i) = max_position_limit_[i]*D2R;
    //ROS_INFO("joint [%d] max: %f", i, max_joint_position_limit(i));
  }

//Solvers
  rleg_fk_solver = new KDL::ChainFkSolverPos_recursive(rleg_chain);
  rleg_ik_vel_solver = new KDL::ChainIkSolverVel_pinv(rleg_chain);
  rleg_ik_pos_solver = new KDL::ChainIkSolverPos_LMA(rleg_chain,
                                                        //min_joint_position_limit, max_joint_position_limit,
                                                        //*rleg_fk_solver,
                                                        //*rleg_ik_vel_solver,
                                                        1E-5,
                                                        300
                                                        );

  lleg_fk_solver = new KDL::ChainFkSolverPos_recursive(lleg_chain);
  lleg_ik_vel_solver = new KDL::ChainIkSolverVel_pinv(lleg_chain);
  lleg_ik_pos_solver = new KDL::ChainIkSolverPos_LMA(lleg_chain,
                                                        //min_joint_position_limit, max_joint_position_limit,
                                                        //*lleg_fk_solver,
                                                        //*lleg_ik_vel_solver,
                                                        1E-5,
                                                        300
                                                        );

}

op3_kdl_move_pelvis::~op3_kdl_move_pelvis(){
  delete rleg_fk_solver;
  delete rleg_ik_vel_solver;
  delete rleg_ik_pos_solver;

  delete lleg_fk_solver;
  delete lleg_ik_vel_solver;
  delete lleg_ik_pos_solver;
}

void op3_kdl_move_pelvis::setJointPosition(Eigen::VectorXd rleg_joint_position_, Eigen::VectorXd lleg_joint_position_){

  rleg_joint_position.data = rleg_joint_position_;
  lleg_joint_position.data = lleg_joint_position_;

}

void op3_kdl_move_pelvis::solveForwardKinematics(KDL::Frame & body_pose){

///Maybe realize some check of equality for left and right legs
  rleg_fk_solver->JntToCart(rleg_joint_position,body_pose);
  //lleg_fk_solver->JntToCart(lleg_joint_position,body_pose);

  ROS_INFO("Body position x:%f, y:%f, z:%f", body_pose.p.x(),body_pose.p.y(),body_pose.p.z());

  double roll, pitch, yaw;
  body_pose.M.GetRPY(roll,pitch,yaw);
  ROS_INFO("Body orientation R:%f, P:%f, Y:%f", roll, pitch, yaw);
}

bool op3_kdl_move_pelvis::solveInverseKinematics(KDL::Frame pelvis_pose,
                                                    Eigen::VectorXd & rleg_des_joint_pos_, Eigen::VectorXd & lleg_des_joint_pos_){

  rleg_des_joint_pos_.resize(JOINT_NUM);
  lleg_des_joint_pos_.resize(JOINT_NUM);

  int ik_pose_err = rleg_ik_pos_solver->CartToJnt(rleg_joint_position, pelvis_pose, rleg_des_joint_pos);

  if (ik_pose_err != 0)
  {
    ROS_WARN("RIGH LEG IK ERROR : %s", rleg_ik_pos_solver->strError(ik_pose_err));
    //return false;
  }
  else {
    for (int i=0; i<JOINT_NUM;i++){
      rleg_des_joint_pos_(i) = rleg_des_joint_pos(i);
    }
    ROS_INFO("Right leg (deg) an_r:%f, an_p:%f, kn_p:%f, hip_p:%f, hip_r:%f, hip_yaw:%f",
             //r_an_r*R2D,r_an_p*R2D,r_kn_p*R2D,
             //r_hp_p*R2D,r_hp_r*R2D,r_hp_y*R2D
             rleg_des_joint_pos_[0]*R2D,rleg_des_joint_pos_[1]*R2D,rleg_des_joint_pos_[2]*R2D,
             rleg_des_joint_pos_[3]*R2D,rleg_des_joint_pos_[4]*R2D,rleg_des_joint_pos_[5]*R2D
             );
  }

  //double r_an_r = (rleg_des_joint_pos_[0]/M_PI - (int)(rleg_des_joint_pos_[0]/M_PI))*M_PI;
  //double r_an_p = (rleg_des_joint_pos_[1]/M_PI - (int)(rleg_des_joint_pos_[1]/M_PI))*M_PI;
  //double r_kn_p = (rleg_des_joint_pos_[2]/M_PI - (int)(rleg_des_joint_pos_[2]/M_PI))*M_PI;
  //double r_hp_p = (rleg_des_joint_pos_[3]/M_PI - (int)(rleg_des_joint_pos_[3]/M_PI))*M_PI;
  //double r_hp_r = (rleg_des_joint_pos_[4]/M_PI - (int)(rleg_des_joint_pos_[4]/M_PI))*M_PI;
  //double r_hp_y = (rleg_des_joint_pos_[5]/M_PI - (int)(rleg_des_joint_pos_[5]/M_PI))*M_PI;




  ik_pose_err = lleg_ik_pos_solver->CartToJnt(lleg_joint_position, pelvis_pose, lleg_des_joint_pos);
  if (ik_pose_err != 0)
  {
    ROS_WARN("LEFT LEG IK ERR : %s", lleg_ik_pos_solver->strError(ik_pose_err));
    return false;
  }
  else {
    for (int i=0; i<JOINT_NUM;i++){
      lleg_des_joint_pos_(i) = lleg_des_joint_pos(i);
    }
    ROS_INFO(" Left leg (deg) an_r:%f, an_p:%f, kn_p:%f, hip_p:%f, hip_r:%f, hip_yaw:%f",
                        lleg_des_joint_pos_[0]*R2D,lleg_des_joint_pos_[1]*R2D,lleg_des_joint_pos_[2]*R2D,
                        lleg_des_joint_pos_[3]*R2D,lleg_des_joint_pos_[4]*R2D,lleg_des_joint_pos_[5]*R2D);
  }

  return true;
}






















