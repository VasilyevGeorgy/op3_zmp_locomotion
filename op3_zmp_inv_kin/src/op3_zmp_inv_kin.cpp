#include "op3_zmp_inv_kin.h"

op3_zmp_inv_kin::op3_zmp_inv_kin()
{

  rleg_joint_pos.resize(JOINT_NUM);
  lleg_joint_pos.resize(JOINT_NUM);

  rleg_min_pos_limit.resize(JOINT_NUM);
  rleg_max_pos_limit.resize(JOINT_NUM);
  lleg_min_pos_limit.resize(JOINT_NUM);
  lleg_max_pos_limit.resize(JOINT_NUM);


  for (int i=0; i < JOINT_NUM; i++){
    rleg_joint_pos(i) = 0.0;
    lleg_joint_pos(i) = 0.0;

    rleg_min_pos_limit(i) = 0.0;
    rleg_max_pos_limit(i) = 0.0;

    lleg_min_pos_limit(i) = 0.0;
    lleg_max_pos_limit(i) = 0.0;


  }

}

op3_zmp_inv_kin::~op3_zmp_inv_kin(){

  delete rleg_fk_solver;
  delete rleg_ik_vel_solver;
  delete rleg_ik_pos_solver;

  delete lleg_fk_solver;
  delete lleg_ik_vel_solver;
  delete lleg_ik_pos_solver;

}

void op3_zmp_inv_kin::initializeROS(){
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
}

void op3_zmp_inv_kin::publishMessageROS(Eigen::VectorXd rleg_jnt_angle_, Eigen::VectorXd lleg_jnt_angle_){

  r_an_r_msg.data =  rleg_jnt_angle_(5);
  //ankle_roll = -lleg_jnt_angle_(0);
  r_an_p_msg.data =  rleg_jnt_angle_(4);
  r_kn_p_msg.data =  rleg_jnt_angle_(3);
  r_hip_p_msg.data = rleg_jnt_angle_(2);
  r_hip_r_msg.data = rleg_jnt_angle_(1);
  //hip_roll = -lleg_jnt_angle_(4);
  r_hip_y_msg.data = rleg_jnt_angle_(0);

  r_hip_y_pub.publish(r_hip_y_msg);
  r_hip_r_pub.publish(r_hip_r_msg);
  r_hip_p_pub.publish(r_hip_p_msg);
  r_kn_p_pub.publish(r_kn_p_msg);
  r_an_p_pub.publish(r_an_p_msg);
  r_an_r_pub.publish(r_an_r_msg);

  l_an_r_msg.data =  lleg_jnt_angle_(5);
  //ankle_roll = -lleg_jnt_angle_(0);
  l_an_p_msg.data =  lleg_jnt_angle_(4);
  l_kn_p_msg.data =  lleg_jnt_angle_(3);
  l_hip_p_msg.data = lleg_jnt_angle_(2);
  l_hip_r_msg.data = lleg_jnt_angle_(1);
  //hip_roll = -lleg_jnt_angle_(4);
  l_hip_y_msg.data = lleg_jnt_angle_(0);

  l_hip_y_pub.publish(l_hip_y_msg);
  l_hip_r_pub.publish(l_hip_r_msg);
  l_hip_p_pub.publish(l_hip_p_msg);
  l_kn_p_pub.publish(l_kn_p_msg);
  l_an_p_pub.publish(l_an_p_msg);
  l_an_r_pub.publish(l_an_r_msg);

}

void op3_zmp_inv_kin::initialization(KDL::Frame pelvis_pose){

  this->initializeChains(pelvis_pose);

  //Set joint limits
  std::vector<double> rleg_min_pos_limit_, rleg_max_pos_limit_;
  rleg_min_pos_limit_.push_back(-90.0); rleg_max_pos_limit_.push_back(90.0); // hip_y
  rleg_min_pos_limit_.push_back(-90.0);	rleg_max_pos_limit_.push_back(90.0); // hip_r
  rleg_min_pos_limit_.push_back(-90.0); rleg_max_pos_limit_.push_back(90.0); // hip_p
  rleg_min_pos_limit_.push_back(-180.0);	rleg_max_pos_limit_.push_back(0.01); // kn_p  -180.0 0.1
  rleg_min_pos_limit_.push_back(-90.0);	rleg_max_pos_limit_.push_back(90.0); // an_p
  rleg_min_pos_limit_.push_back(-90.0); rleg_max_pos_limit_.push_back(90.0); // an_r

  std::vector<double> lleg_min_pos_limit_, lleg_max_pos_limit_;
  lleg_min_pos_limit_.push_back(-90.0); lleg_max_pos_limit_.push_back(90.0); // hip_y
  lleg_min_pos_limit_.push_back(-90.0);	lleg_max_pos_limit_.push_back(90.0); // hip_r
  lleg_min_pos_limit_.push_back(-90.0); lleg_max_pos_limit_.push_back(90.0); // hip_p
  lleg_min_pos_limit_.push_back(-0.01);	lleg_max_pos_limit_.push_back(180.0); // kn_p
  lleg_min_pos_limit_.push_back(-90.0);	lleg_max_pos_limit_.push_back(90.0); // an_p
  lleg_min_pos_limit_.push_back(-90.0); lleg_max_pos_limit_.push_back(90.0); // an_r

  for (int i=0; i<JOINT_NUM; i++)
  {
    //std::cout<<i<<std::endl;
    rleg_min_pos_limit(i) = rleg_min_pos_limit_[i]*D2R; //D2R - degrees to radians
    rleg_max_pos_limit(i) = rleg_max_pos_limit_[i]*D2R;
    //ROS_INFO("joint [%d] max: %f", i, max_joint_pos_limit(i));

    lleg_min_pos_limit(i) = lleg_min_pos_limit_[i]*D2R; //D2R - degrees to radians
    lleg_max_pos_limit(i) = lleg_max_pos_limit_[i]*D2R;

    //ROS_INFO("lleg_min_pos_limit(%d): %f;   lleg_max_pos_limit(%d): %f",
    //         i,lleg_min_pos_limit(i)*R2D,i,lleg_max_pos_limit(i)*R2D);

  }

  //Setup solvers
  rleg_fk_solver = new KDL::ChainFkSolverPos_recursive(rleg_chain);
  rleg_ik_vel_solver  = new KDL::ChainIkSolverVel_pinv(rleg_chain);
  rleg_ik_pos_solver = new KDL::ChainIkSolverPos_NR_JL(rleg_chain,
                                                           rleg_min_pos_limit, rleg_max_pos_limit,
                                                           *rleg_fk_solver,
                                                           *rleg_ik_vel_solver
                                                           );

  lleg_fk_solver = new KDL::ChainFkSolverPos_recursive(lleg_chain);
  lleg_ik_vel_solver  = new KDL::ChainIkSolverVel_pinv(lleg_chain);
  lleg_ik_pos_solver = new KDL::ChainIkSolverPos_NR_JL(lleg_chain,
                                                           lleg_min_pos_limit, lleg_max_pos_limit,
                                                           *lleg_fk_solver,
                                                           *lleg_ik_vel_solver
                                                           );

}

void op3_zmp_inv_kin::setJointPosition(Eigen::VectorXd rleg_joint_position_, Eigen::VectorXd lleg_joint_position_){

  rleg_joint_pos.data = rleg_joint_position_;
  lleg_joint_pos.data = lleg_joint_position_;

}

bool op3_zmp_inv_kin::movePelvis(KDL::Frame leg_des_pose, Eigen::VectorXd &leg_des_joint_pos_, std::string legType){

  //leg_des_joint_pos_.resize(JOINT_NUM);

  //Distance between start and end points
  double dx = leg_des_pose.p.x() - pelvis_pose.p.x();
  double dy = leg_des_pose.p.y() - pelvis_pose.p.y();
  double dz = leg_des_pose.p.z() - pelvis_pose.p.z();

  double roll_, pitch_, yaw_;
  leg_des_pose.M.GetRPY(roll_,pitch_,yaw_);

  std::transform(legType.begin(),legType.end(),legType.begin(), ::tolower);

  //Foot translation --> pelvis translation
  KDL::Frame pseudo;
  int ik_pose_err = 0;

  if (legType == "right"){
    pseudo = KDL::Frame(KDL::Rotation::RPY(-roll_,-pitch_,-yaw_),
                           KDL::Vector(rfoot_pose.p.x()-dx, rfoot_pose.p.y()-dy, rfoot_pose.p.z()-dz)
                        );

    ik_pose_err = rleg_ik_pos_solver->CartToJnt(rleg_joint_pos, pseudo, rleg_des_joint_pos);

    if (ik_pose_err != 0)
    {
      ROS_WARN("RIGHT LEG IK ERROR : %s", rleg_ik_pos_solver->strError(ik_pose_err));

      return false;
    }
    else {
      for (int i=0; i<JOINT_NUM;i++){
          leg_des_joint_pos_(i) = rleg_des_joint_pos(i);
      }
      ROS_INFO("Right leg (deg) hip_yaw:%f, hip_r:%f, hip_p:%f, kn_p:%f, an_p :%f, an_r:%f",
               leg_des_joint_pos_(0)*R2D,leg_des_joint_pos_(1)*R2D,leg_des_joint_pos_(2)*R2D,
               leg_des_joint_pos_(3)*R2D,leg_des_joint_pos_(4)*R2D,leg_des_joint_pos_(5)*R2D
               );
    }
  }
  else{
    if(legType == "left"){
      pseudo = KDL::Frame(KDL::Rotation::RPY(-roll_,-pitch_,-yaw_),
                             KDL::Vector(lfoot_pose.p.x()-dx, lfoot_pose.p.y()-dy, lfoot_pose.p.z()-dz)
                          );

      ik_pose_err = lleg_ik_pos_solver->CartToJnt(lleg_joint_pos, pseudo, lleg_des_joint_pos);

      if (ik_pose_err != 0)
      {
        ROS_WARN("LEFT LEG IK ERROR : %s", lleg_ik_pos_solver->strError(ik_pose_err));

        return false;
      }
      else {
        for (int i=0; i<JOINT_NUM;i++){
            leg_des_joint_pos_(i) = lleg_des_joint_pos(i);
        }
        ROS_INFO("Left leg (deg) hip_yaw:%f, hip_r:%f, hip_p:%f, kn_p:%f, an_p :%f, an_r:%f",
                 leg_des_joint_pos_(0)*R2D,leg_des_joint_pos_(1)*R2D,leg_des_joint_pos_(2)*R2D,
                 leg_des_joint_pos_(3)*R2D,leg_des_joint_pos_(4)*R2D,leg_des_joint_pos_(5)*R2D
                 );
      }
    }
    else{
      ROS_WARN("INCORRECT INPUT FOR LEG TYPE");
      return false;
    }
  }

  std::cout<<"pseudo_z = "<<pseudo.p.z()<<std::endl;

}

void op3_zmp_inv_kin::goToInitialPose(KDL::Frame pelvis_des_pose){

  ROS_INFO("Test0!");

  pelvis_pose = KDL::Frame(
                    KDL::Rotation::RPY(0.0,0.0,0.0),
                    KDL::Vector(0.0,0.0,0.3697) // z - max height
                       );

  //Initialization
  this->initialization(pelvis_pose);


  rfoot_pose = KDL::Frame(KDL::Rotation::RPY(0.0,0.0,0.0),
                          KDL::Vector(0.0,-y_offset,0.0)
                           );
  lfoot_pose = KDL::Frame(KDL::Rotation::RPY(0.0,0.0,0.0),
                          KDL::Vector(0.0,y_offset,0.0)
                           );

  Eigen::VectorXd rleg_joint_pos_;
  Eigen::VectorXd lleg_joint_pos_;

  rleg_joint_pos_.resize(JOINT_NUM);
  rleg_joint_pos_(0) =  0.001;
  rleg_joint_pos_(1) = -0.001;
  rleg_joint_pos_(2) =  0.001;
  rleg_joint_pos_(3) = -0.001;
  rleg_joint_pos_(4) =  0.001;
  rleg_joint_pos_(5) = -0.001;

  lleg_joint_pos_.resize(JOINT_NUM);
  lleg_joint_pos_(0) = 0.001;
  lleg_joint_pos_(1) = 0.001;
  lleg_joint_pos_(2) = 0.001;
  lleg_joint_pos_(3) = 0.001;
  lleg_joint_pos_(4) = 0.001;
  lleg_joint_pos_(5) = 0.001;

  this->setJointPosition(rleg_joint_pos_,lleg_joint_pos_);

  ROS_INFO("Test1!");

  double freq = 1000; // in HZ
  ros::Rate rate(freq);
  double time = 10; // in sec
  int numOfSteps = int (freq*time);

  double dz = (pelvis_des_pose.p.z()-pelvis_pose.p.z())/numOfSteps;

  std::cout<<"pelvis_des_pose.p.z() = "<<pelvis_des_pose.p.z()<<std::endl;
  std::cout<<"pelvis_pose.p.z() = "<<pelvis_pose.p.z()<<std::endl;
  std::cout<<"dz = "<<dz<<std::endl;

  this->initializeROS();

  KDL::Frame pos = pelvis_pose;

  for(int i=0;i<numOfSteps;i++){
  //while (ros::ok()){
    pos.p.data[2] += dz;
    std::cout<<"pos.p.z() = "<<pos.p.z()<<std::endl;

    this->movePelvis(pos, rleg_joint_pos_, "Right");
    this->movePelvis(pos, lleg_joint_pos_, "Left");

    this->publishMessageROS(rleg_joint_pos_, lleg_joint_pos_);
    this->setJointPosition(rleg_joint_pos_, lleg_joint_pos_);

    //pelvis_pose = pos;

    rate.sleep();
  }



  ROS_INFO("Test2!");

}
