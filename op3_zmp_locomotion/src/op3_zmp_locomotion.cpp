#include "op3_zmp_locomotion.h"

op3_zmp_locomotion::op3_zmp_locomotion(){

  rleg_joint_position.resize(JOINT_NUM);
  lleg_joint_position.resize(JOINT_NUM);

  for (int i=0; i < JOINT_NUM; i++){
    rleg_joint_position(i) = 0.0;
    lleg_joint_position(i) = 0.0;
  }

  //r_hip_y_sub = node.subscribe("robotis_op3/r_hip_yaw_position/command",10, &op3_zmp_locomotion::r_hip_y_callback, this);
  //r_hip_y_sub = node.subscribe("robotis_op3/r_hip_roll_position/command",10, &op3_zmp_locomotion::r_hip_r_callback, this);
  //r_hip_y_sub = node.subscribe("robotis_op3/r_hip_pitch_position/command",10, &op3_zmp_locomotion::r_hip_p_callback, this);
  //r_hip_y_sub = node.subscribe("robotis_op3/r_knee_position/command",10, &op3_zmp_locomotion::r_kn_p_callback, this);
  //r_hip_y_sub = node.subscribe("robotis_op3/r_ank_pitch_position/command",10, &op3_zmp_locomotion::r_an_p_callback, this);
  //r_hip_y_sub = node.subscribe("robotis_op3/r_ank_roll_position/command",10, &op3_zmp_locomotion::r_an_r_callback, this);

//  ROS_INFO("!!!%f", cur_r_kn_p);

//Right leg
  r_hip_y_pub = node.advertise<std_msgs::Float64>("robotis_op3/r_hip_yaw_position/command",10);
  r_hip_r_pub = node.advertise<std_msgs::Float64>("robotis_op3/r_hip_roll_position/command",10);
  r_hip_p_pub = node.advertise<std_msgs::Float64>("robotis_op3/r_hip_pitch_position/command",10);
  r_kn_p_pub = node.advertise<std_msgs::Float64>("robotis_op3/r_knee_position/command",10);
  r_an_p_pub = node.advertise<std_msgs::Float64>("robotis_op3/r_ank_pitch_position/command",10);
  r_an_r_pub = node.advertise<std_msgs::Float64>("robotis_op3/r_ank_roll_position/command",10);
//Left leg
  l_hip_y_pub = node.advertise<std_msgs::Float64>("robotis_op3/l_hip_yaw_position/command",10);
  l_hip_r_pub = node.advertise<std_msgs::Float64>("robotis_op3/l_hip_roll_position/command",10);
  l_hip_p_pub = node.advertise<std_msgs::Float64>("robotis_op3/l_hip_pitch_position/command",10);
  l_kn_p_pub = node.advertise<std_msgs::Float64>("robotis_op3/l_knee_position/command",10);
  l_an_p_pub = node.advertise<std_msgs::Float64>("robotis_op3/l_ank_pitch_position/command",10);
  l_an_r_pub = node.advertise<std_msgs::Float64>("robotis_op3/l_ank_roll_position/command",10);

  //std_msgs::Float64 cur_r_hip_y_ = *(ros::topic::waitForMessage<std_msgs::Float64>("robotis_op3/r_hip_yaw_position/command"));
  //cur_r_hip_y = cur_r_hip_y_.data;
  //ROS_INFO("\n\n\n %f \n\n\n", cur_r_hip_y);

  init_com_transl = true;


}

op3_zmp_locomotion::~op3_zmp_locomotion(){

  delete rleg_pelvis_to_foot_fk_solver;
  delete rleg_pelvis_to_foot_ik_vel_solver;
  delete rleg_pelvis_to_foot_ik_pos_solver;

  delete rleg_foot_to_pelvis_fk_solver;
  delete rleg_foot_to_pelvis_ik_vel_solver;
  delete rleg_foot_to_pelvis_ik_pos_solver;

  delete lleg_pelvis_to_foot_fk_solver;
  delete lleg_pelvis_to_foot_ik_vel_solver;
  delete lleg_pelvis_to_foot_ik_pos_solver;

  delete lleg_foot_to_pelvis_fk_solver;
  delete lleg_foot_to_pelvis_ik_vel_solver;
  delete lleg_foot_to_pelvis_ik_pos_solver;

}



void op3_zmp_locomotion::r_hip_y_callback(const std_msgs::Float64::ConstPtr &angle)
{
  cur_r_hip_y = angle->data;
  ROS_INFO("!-!%f", angle->data);
  ROS_INFO("!!!%f", cur_r_hip_y);
  //r_hip_y_sub.shutdown();
}
void op3_zmp_locomotion::r_hip_r_callback(const std_msgs::Float64::ConstPtr &angle)
{
  cur_r_hip_r = angle->data;
  ROS_INFO("!!!%f", cur_r_hip_r);
  //r_hip_y_sub.shutdown();
}
void op3_zmp_locomotion::r_hip_p_callback(const std_msgs::Float64::ConstPtr &angle)
{
  cur_r_hip_p = angle->data;
  ROS_INFO("!!!%f", cur_r_hip_p);
  //r_hip_y_sub.shutdown();
}
void op3_zmp_locomotion::r_kn_p_callback(const std_msgs::Float64::ConstPtr &angle)
{
  cur_r_kn_p = angle->data;
  ROS_INFO("!!!%f", cur_r_kn_p);
  //r_hip_y_sub.shutdown();
}
void op3_zmp_locomotion::r_an_p_callback(const std_msgs::Float64::ConstPtr &angle)
{
  cur_r_an_p= angle->data;
  ROS_INFO("!!!%f", cur_r_an_p);
  //r_hip_y_sub.shutdown();
}
void op3_zmp_locomotion::r_an_r_callback(const std_msgs::Float64::ConstPtr &angle)
{
  cur_r_an_r = angle->data;
  ROS_INFO("!!!%f", cur_r_an_r);
  //r_hip_y_sub.shutdown();
}


void op3_zmp_locomotion::initialize(KDL::Frame pelvis_pose, KDL::Frame rfoot_pose, KDL::Frame lfoot_pose){

  rleg_current_pose = rfoot_pose;
  lleg_current_pose = lfoot_pose;

//Rigth leg chains
  this->op3_right_leg(pelvis_pose, rfoot_pose);
//Left leg
  this->op3_left_leg(pelvis_pose, lfoot_pose);

// Set Joint Limits
  std::vector<double> rleg_foot_to_pelvis_min_position_limit_, rleg_foot_to_pelvis_max_position_limit_;
  rleg_foot_to_pelvis_min_position_limit_.push_back(-90.0); rleg_foot_to_pelvis_max_position_limit_.push_back(90.0); // an_r
  rleg_foot_to_pelvis_min_position_limit_.push_back(-90.0);	rleg_foot_to_pelvis_max_position_limit_.push_back(90.0); // an_p
  rleg_foot_to_pelvis_min_position_limit_.push_back(-180.0); rleg_foot_to_pelvis_max_position_limit_.push_back(-0.01); // kn_p
  rleg_foot_to_pelvis_min_position_limit_.push_back(-10.0);	rleg_foot_to_pelvis_max_position_limit_.push_back(180.0); // hip_p
  rleg_foot_to_pelvis_min_position_limit_.push_back(-90.0);	rleg_foot_to_pelvis_max_position_limit_.push_back(90.0); // hip_r
  rleg_foot_to_pelvis_min_position_limit_.push_back(-90.0); rleg_foot_to_pelvis_max_position_limit_.push_back(90.0); // hip_y

  rleg_foot_to_pelvis_min_position_limit.resize(JOINT_NUM);
  rleg_foot_to_pelvis_max_position_limit.resize(JOINT_NUM);

  std::vector<double> lleg_foot_to_pelvis_min_position_limit_, lleg_foot_to_pelvis_max_position_limit_;
  lleg_foot_to_pelvis_min_position_limit_.push_back(-90.0); lleg_foot_to_pelvis_max_position_limit_.push_back(90.0); // an_r
  lleg_foot_to_pelvis_min_position_limit_.push_back(-90.0);	lleg_foot_to_pelvis_max_position_limit_.push_back(90.0); // an_p
  lleg_foot_to_pelvis_min_position_limit_.push_back(-0.01); lleg_foot_to_pelvis_max_position_limit_.push_back(180.0); // kn_p
  lleg_foot_to_pelvis_min_position_limit_.push_back(-180.0);	lleg_foot_to_pelvis_max_position_limit_.push_back(10.0); // hip_p
  lleg_foot_to_pelvis_min_position_limit_.push_back(-90.0);	lleg_foot_to_pelvis_max_position_limit_.push_back(90.0); // hip_r
  lleg_foot_to_pelvis_min_position_limit_.push_back(-90.0); lleg_foot_to_pelvis_max_position_limit_.push_back(90.0); // hip_y

  lleg_foot_to_pelvis_min_position_limit.resize(JOINT_NUM);
  lleg_foot_to_pelvis_max_position_limit.resize(JOINT_NUM);

  rleg_pelvis_to_foot_min_position_limit.resize(JOINT_NUM);
  rleg_pelvis_to_foot_max_position_limit.resize(JOINT_NUM);
  lleg_pelvis_to_foot_min_position_limit.resize(JOINT_NUM);
  lleg_pelvis_to_foot_max_position_limit.resize(JOINT_NUM);

  std::vector<double> lleg_pelvis_to_foot_min_position_limit_, lleg_pelvis_to_foot_max_position_limit_;
  lleg_pelvis_to_foot_min_position_limit_.push_back(-90.0); lleg_pelvis_to_foot_max_position_limit_.push_back(90.0); // hip_y
  lleg_pelvis_to_foot_min_position_limit_.push_back(-90.0);	lleg_pelvis_to_foot_max_position_limit_.push_back(90.0); // hip_r
  lleg_pelvis_to_foot_min_position_limit_.push_back(-90.0); lleg_pelvis_to_foot_max_position_limit_.push_back(90.0); // hip_p
  lleg_pelvis_to_foot_min_position_limit_.push_back(-0.01);	lleg_pelvis_to_foot_max_position_limit_.push_back(90.0); // kn_p
  lleg_pelvis_to_foot_min_position_limit_.push_back(-90.0);	lleg_pelvis_to_foot_max_position_limit_.push_back(90.0); // an_p
  lleg_pelvis_to_foot_min_position_limit_.push_back(-90.0); lleg_pelvis_to_foot_max_position_limit_.push_back(90.0); // an_r


  for (int i=0; i<JOINT_NUM; i++)
  {
    rleg_foot_to_pelvis_min_position_limit(i) = rleg_foot_to_pelvis_min_position_limit_[i]*D2R; //D2R - degrees to radians
    rleg_foot_to_pelvis_max_position_limit(i) = rleg_foot_to_pelvis_max_position_limit_[i]*D2R;
    //ROS_INFO("joint [%d] max: %f", i, max_joint_position_limit(i));
    rleg_pelvis_to_foot_min_position_limit(i) = rleg_foot_to_pelvis_min_position_limit_[(JOINT_NUM-1)-i]*D2R;
    rleg_pelvis_to_foot_max_position_limit(i) = rleg_foot_to_pelvis_max_position_limit_[(JOINT_NUM-1)-i]*D2R;

    lleg_foot_to_pelvis_min_position_limit(i) = lleg_foot_to_pelvis_min_position_limit_[i]*D2R; //D2R - degrees to radians
    lleg_foot_to_pelvis_max_position_limit(i) = lleg_foot_to_pelvis_max_position_limit_[i]*D2R;

    lleg_pelvis_to_foot_min_position_limit(i) = lleg_pelvis_to_foot_min_position_limit_[i]*D2R;
    lleg_pelvis_to_foot_max_position_limit(i) = lleg_pelvis_to_foot_max_position_limit_[i]*D2R;


    //lleg_pelvis_to_foot_min_position_limit(i) = lleg_foot_to_pelvis_max_position_limit_[(JOINT_NUM-1)-i]*D2R;
    //lleg_pelvis_to_foot_max_position_limit(i) = lleg_foot_to_pelvis_min_position_limit_[(JOINT_NUM-1)-i]*D2R;
    //
    //
    //ROS_INFO("lleg_pelvis_to_foot_min_position_limit(%d): %f;   lleg_pelvis_to_foot_max_position_limit(%d): %f",
    //         i,lleg_pelvis_to_foot_min_position_limit(i)*R2D,i,lleg_pelvis_to_foot_max_position_limit(i)*R2D);

  }

//Solvers

  rleg_pelvis_to_foot_fk_solver = new KDL::ChainFkSolverPos_recursive(rleg_pelvis_to_foot_chain);
  rleg_pelvis_to_foot_ik_vel_solver = new KDL::ChainIkSolverVel_pinv(rleg_pelvis_to_foot_chain);
  rleg_pelvis_to_foot_ik_pos_solver = new KDL::ChainIkSolverPos_NR_JL(rleg_pelvis_to_foot_chain,
                                                                         rleg_pelvis_to_foot_min_position_limit, rleg_pelvis_to_foot_max_position_limit,
                                                                         *rleg_pelvis_to_foot_fk_solver,
                                                                         *rleg_pelvis_to_foot_ik_vel_solver
                                                                         );


  rleg_foot_to_pelvis_fk_solver = new KDL::ChainFkSolverPos_recursive(rleg_foot_to_pelvis_chain);
  rleg_foot_to_pelvis_ik_vel_solver = new KDL::ChainIkSolverVel_pinv(rleg_foot_to_pelvis_chain);
  rleg_foot_to_pelvis_ik_pos_solver = new KDL::ChainIkSolverPos_NR_JL(rleg_foot_to_pelvis_chain,
                                                                         rleg_foot_to_pelvis_min_position_limit, rleg_foot_to_pelvis_max_position_limit,
                                                                         *rleg_foot_to_pelvis_fk_solver,
                                                                         *rleg_foot_to_pelvis_ik_vel_solver
                                                                         );

  //lleg_pelvis_to_foot_ik_pos_solver = new KDL::ChainIkSolverPos_LMA(lleg_pelvis_to_foot_chain,
  //                                                                     1E-5,
  //                                                                     300);

  lleg_pelvis_to_foot_fk_solver = new KDL::ChainFkSolverPos_recursive(lleg_pelvis_to_foot_chain);
  lleg_pelvis_to_foot_ik_vel_solver = new KDL::ChainIkSolverVel_pinv(lleg_pelvis_to_foot_chain);
  lleg_pelvis_to_foot_ik_pos_solver = new KDL::ChainIkSolverPos_NR_JL(lleg_pelvis_to_foot_chain,
                                                                         lleg_pelvis_to_foot_min_position_limit, lleg_pelvis_to_foot_max_position_limit,
                                                                         *lleg_pelvis_to_foot_fk_solver,
                                                                         *lleg_pelvis_to_foot_ik_vel_solver
                                                                         );

  lleg_foot_to_pelvis_fk_solver = new KDL::ChainFkSolverPos_recursive(lleg_foot_to_pelvis_chain);
  lleg_foot_to_pelvis_ik_vel_solver = new KDL::ChainIkSolverVel_pinv(lleg_foot_to_pelvis_chain);
  lleg_foot_to_pelvis_ik_pos_solver = new KDL::ChainIkSolverPos_NR_JL(lleg_foot_to_pelvis_chain,
                                                                         lleg_foot_to_pelvis_min_position_limit, lleg_foot_to_pelvis_max_position_limit,
                                                                         *lleg_foot_to_pelvis_fk_solver,
                                                                         *lleg_foot_to_pelvis_ik_vel_solver
                                                                         );


}

void op3_zmp_locomotion::setJointPosition(Eigen::VectorXd rleg_joint_position_, Eigen::VectorXd lleg_joint_position_){

  rleg_joint_position.data = rleg_joint_position_;
  lleg_joint_position.data = lleg_joint_position_;

}

bool op3_zmp_locomotion::moveCOMToRightLeg(KDL::Frame pelvis_des_pose, Eigen::VectorXd &rleg_des_joint_pos_){

  rleg_des_joint_pos_.resize(JOINT_NUM);

  int ik_pose_err = rleg_foot_to_pelvis_ik_pos_solver->CartToJnt(rleg_joint_position, pelvis_des_pose, rleg_des_joint_pos);

  if (ik_pose_err != 0)
  {
    ROS_WARN("RIGHT LEG IK ERROR : %s", rleg_foot_to_pelvis_ik_pos_solver->strError(ik_pose_err));
    for (int i=0; i<JOINT_NUM;i++){
      //rleg_des_joint_pos_(i) = rleg_joint_position(i);
    }
    return false;
  }
  else {
    for (int i=0; i<JOINT_NUM;i++){
      //if (fabs(rleg_des_joint_pos(i))<2*M_PI)
        rleg_des_joint_pos_(i) = rleg_des_joint_pos(i);
    }
    ROS_INFO("Right leg (deg) an_r:%f, an_p:%f, kn_p:%f, hip_p:%f, hip_r:%f, hip_yaw:%f",
             rleg_des_joint_pos_(0)*R2D,rleg_des_joint_pos_(1)*R2D,rleg_des_joint_pos_(2)*R2D,
             rleg_des_joint_pos_(3)*R2D,rleg_des_joint_pos_(4)*R2D,rleg_des_joint_pos_(5)*R2D
             );
  }

  return true;
}

bool op3_zmp_locomotion::moveCOMToLeftLeg(KDL::Frame pelvis_des_pose, Eigen::VectorXd &lleg_des_joint_pos_){

  lleg_des_joint_pos_.resize(JOINT_NUM);

  int ik_pose_err = lleg_foot_to_pelvis_ik_pos_solver->CartToJnt(lleg_joint_position, pelvis_des_pose, lleg_des_joint_pos);

  if (ik_pose_err != 0)
  {
    ROS_WARN(" LEFT LEG IK ERROR : %s", lleg_foot_to_pelvis_ik_pos_solver->strError(ik_pose_err));
    for (int i=0; i<JOINT_NUM;i++){
      lleg_des_joint_pos_(i) = lleg_joint_position(i);
    }
    return false;
  }
  else {
    for (int i=0; i<JOINT_NUM;i++){
      lleg_des_joint_pos_(i) = lleg_joint_position(i);
      //ROS_INFO("Move CoM to left leg test!!!");
      lleg_des_joint_pos_(i) = lleg_des_joint_pos(i);
    }
    ROS_INFO("Left leg (deg) an_r:%f, an_p:%f, kn_p:%f, hip_p:%f, hip_r:%f, hip_yaw:%f",
             lleg_des_joint_pos_(0)*R2D,lleg_des_joint_pos_(1)*R2D,lleg_des_joint_pos_(2)*R2D,
             lleg_des_joint_pos_(3)*R2D,lleg_des_joint_pos_(4)*R2D,lleg_des_joint_pos_(5)*R2D
             );
  }

  return true;
}

bool op3_zmp_locomotion::moveRightLeg(KDL::Frame rleg_des_pose, Eigen::VectorXd &rleg_des_joint_pos_){

  int ik_pose_err = rleg_pelvis_to_foot_ik_pos_solver->CartToJnt(rleg_joint_position, rleg_des_pose, rleg_des_joint_pos);

  if (ik_pose_err != 0)
  {
    ROS_WARN("RIGHT LEG IK ERROR : %s", rleg_foot_to_pelvis_ik_pos_solver->strError(ik_pose_err));
    for (int i=0; i<JOINT_NUM;i++){
      //rleg_des_joint_pos_(i) = rleg_joint_position(i);
    }
    return false;
  }
  else {
    for (int i=0; i<JOINT_NUM;i++){
        rleg_des_joint_pos_(i) = rleg_des_joint_pos(JOINT_NUM-1-i);
    }
    ROS_INFO("Right leg (deg) an_r:%f, an_p:%f, kn_p:%f, hip_p:%f, hip_r:%f, hip_yaw:%f",
             rleg_des_joint_pos_[0]*R2D,rleg_des_joint_pos_[1]*R2D,rleg_des_joint_pos_[2]*R2D,
             rleg_des_joint_pos_[3]*R2D,rleg_des_joint_pos_[4]*R2D,rleg_des_joint_pos_[5]*R2D
             );
  }

  return true;

}

bool op3_zmp_locomotion::moveLeftLeg(KDL::Frame lleg_des_pose, Eigen::VectorXd &lleg_des_joint_pos_){

  int ik_pose_err = lleg_pelvis_to_foot_ik_pos_solver->CartToJnt(lleg_joint_position, lleg_des_pose, lleg_des_joint_pos);

  if (ik_pose_err != 0)
  {
    ROS_WARN("LEFT LEG IK ERROR : %s", lleg_foot_to_pelvis_ik_pos_solver->strError(ik_pose_err));
    for (int i=0; i<JOINT_NUM;i++){
      lleg_des_joint_pos_(i) = lleg_joint_position(i);
    }
    return false;
  }
  else {
    for (int i=0; i<JOINT_NUM;i++){
        //lleg_des_joint_pos_(i) = lleg_joint_position(i);
        lleg_des_joint_pos_(i) = lleg_des_joint_pos(JOINT_NUM-1-i);
    }
    ROS_INFO("Left leg (deg) an_r:%f, an_p:%f, kn_p:%f, hip_p:%f, hip_r:%f, hip_yaw:%f",
             lleg_des_joint_pos_[0]*R2D,lleg_des_joint_pos_[1]*R2D,lleg_des_joint_pos_[2]*R2D,
             lleg_des_joint_pos_[3]*R2D,lleg_des_joint_pos_[4]*R2D,lleg_des_joint_pos_[5]*R2D
             );
  }

  return true;

}


