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

  getCallback = true;

}

op3_zmp_inv_kin::~op3_zmp_inv_kin(){

}

void op3_zmp_inv_kin::deleteChains(){

  delete rleg_chain;
  delete lleg_chain;

}
void op3_zmp_inv_kin::deleteSolvers(){

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

  //leg_joints_pub = node.advertise<sensor_msgs::JointState>("/robotis/direct_control/set_joint_states",100);

}

void op3_zmp_inv_kin::get_joints_pos(){
  present_joint_states_sub = node.subscribe("/robotis/present_joint_states",1000, &op3_zmp_inv_kin::pres_state_callback, this);
  ROS_INFO("CALLBACK TEST 0!");
}

void op3_zmp_inv_kin::pres_state_callback(const sensor_msgs::JointState::ConstPtr &jnt_data)
{

  ROS_INFO("Right leg (deg) hip_yaw:%f, hip_r:%f, hip_p:%f, kn_p:%f, an_p :%f, an_r:%f",
           jnt_data->position[16]*R2D,jnt_data->position[15]*R2D,jnt_data->position[14]*R2D,
           jnt_data->position[17]*R2D,jnt_data->position[11]*R2D,jnt_data->position[12]*R2D
           );

  ROS_INFO("Left leg (deg) hip_yaw:%f, hip_r:%f, hip_p:%f, kn_p:%f, an_p :%f, an_r:%f",
           jnt_data->position[7]*R2D,jnt_data->position[6]*R2D,jnt_data->position[5]*R2D,
           jnt_data->position[8]*R2D,jnt_data->position[2]*R2D,jnt_data->position[3]*R2D
           );

  for (int i=0; i<20; i++){
    std::cout<<jnt_data->name[i]<<" ";
  }
  std::cout<<std::endl;

  rleg_joint_pos(0) = jnt_data->position[16]; // r_hip_yaw
  rleg_joint_pos(1) = jnt_data->position[15]; // r_hip_roll
  rleg_joint_pos(2) = jnt_data->position[14]; // r_hip_pitch
  rleg_joint_pos(3) = jnt_data->position[17]; // r_knee
  rleg_joint_pos(4) = jnt_data->position[11]; // r_ank_pitch
  rleg_joint_pos(5) = jnt_data->position[12]; // r_ank_roll

  lleg_joint_pos(0) = jnt_data->position[7];  // l_hip_yaw
  lleg_joint_pos(1) = jnt_data->position[6];  // l_hip_roll
  lleg_joint_pos(2) = jnt_data->position[5];  // l_hip_pitch
  lleg_joint_pos(3) = jnt_data->position[8];  // l_knee
  lleg_joint_pos(4) = jnt_data->position[2];  // l_ank_pitch
  lleg_joint_pos(5) = jnt_data->position[3];  // l_ank_roll


  getCallback = false;

  ROS_INFO("CALLBACK TEST 1!");


  //present_joint_states_sub.shutdown();

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
  rleg_fk_solver = new KDL::ChainFkSolverPos_recursive(*rleg_chain);
  rleg_ik_vel_solver  = new KDL::ChainIkSolverVel_pinv(*rleg_chain);
  rleg_ik_pos_solver = new KDL::ChainIkSolverPos_NR_JL(*rleg_chain,
                                                           rleg_min_pos_limit, rleg_max_pos_limit,
                                                           *rleg_fk_solver,
                                                           *rleg_ik_vel_solver
                                                           );

  lleg_fk_solver = new KDL::ChainFkSolverPos_recursive(*lleg_chain);
  lleg_ik_vel_solver  = new KDL::ChainIkSolverVel_pinv(*lleg_chain);
  lleg_ik_pos_solver = new KDL::ChainIkSolverPos_NR_JL(*lleg_chain,
                                                           lleg_min_pos_limit, lleg_max_pos_limit,
                                                           *lleg_fk_solver,
                                                           *lleg_ik_vel_solver
                                                           );

}

void op3_zmp_inv_kin::setJointPosition(Eigen::VectorXd rleg_joint_position_, Eigen::VectorXd lleg_joint_position_){

  rleg_joint_pos.data = rleg_joint_position_;
  lleg_joint_pos.data = lleg_joint_position_;

}

bool op3_zmp_inv_kin::movePelvis(KDL::Frame pelvis_des_pose, Eigen::VectorXd &leg_des_joint_pos_, std::string legType){

  //leg_des_joint_pos_.resize(JOINT_NUM);

  //Distance between start and end points
  double delta_x = pelvis_des_pose.p.x() - pelvis_pose.p.x();
  double delta_y = pelvis_des_pose.p.y() - pelvis_pose.p.y();
  double delta_z = pelvis_des_pose.p.z() - pelvis_pose.p.z();

  double roll_, pitch_, yaw_;
  pelvis_des_pose.M.GetRPY(roll_,pitch_,yaw_);

  std::transform(legType.begin(),legType.end(),legType.begin(), ::tolower);

  //Foot translation --> pelvis translation
  KDL::Frame pseudo;
  int ik_pose_err = 0;

  if (legType == "right"){
    pseudo = KDL::Frame(KDL::Rotation::RPY(-roll_,-pitch_,-yaw_),
                           KDL::Vector(rfoot_pose.p.x()-delta_x, rfoot_pose.p.y()-delta_y, rfoot_pose.p.z()-delta_z)
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
                             KDL::Vector(lfoot_pose.p.x()-delta_x, lfoot_pose.p.y()-delta_y, lfoot_pose.p.z()-delta_z)
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

void op3_zmp_inv_kin::InitPoseTest(KDL::Frame pelvis_des_pose){

  ROS_INFO("Test0!");

  pelvis_pose = KDL::Frame(
                    KDL::Rotation::RPY(0.0,0.0,0.0),
                    KDL::Vector(0.0,0.0,0.3697) // z - max height
                       );

  //Initialization
  this->initialization(pelvis_pose);

  this->initializeROS();

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

  //std::cout<<"pelvis_des_pose.p.z() = "<<pelvis_des_pose.p.z()<<std::endl;
  //std::cout<<"pelvis_pose.p.z() = "<<pelvis_pose.p.z()<<std::endl;
  //std::cout<<"dz = "<<dz<<std::endl;

  this->initializeROS();

  KDL::Frame pos = pelvis_pose;

  for(int i=0;i<numOfSteps;i++){
    pos.p.data[2] += dz;
    std::cout<<"pos.p.z() = "<<pos.p.z()<<std::endl;

    this->movePelvis(pos, rleg_joint_pos_, "Right");
    this->movePelvis(pos, lleg_joint_pos_, "Left");

    this->publishMessageROS(rleg_joint_pos_, lleg_joint_pos_);
    this->setJointPosition(rleg_joint_pos_, lleg_joint_pos_);

    //pelvis_pose = pos;

    rate.sleep();
  }

}

bool op3_zmp_inv_kin::launchManager(){

  std::system(RUNMNGRSCRIPT);
  ros::Duration(7.5).sleep();

  return true; // TODO: error processing!

}

void op3_zmp_inv_kin::goToInitialPose(KDL::Frame pelvis_des_pose, stepParam sp){

  ROS_INFO("Test0!");

  pelvis_pose = KDL::Frame(
                    KDL::Rotation::RPY(0.0,0.0,0.0),
                    KDL::Vector(0.0,0.0,0.3697) // z - max height
                       );

  //Initialization
  this->initialization(pelvis_pose);

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

  this->rleg_fk_solver->JntToCart(rleg_joint_pos,rfoot_pose);
  this->lleg_fk_solver->JntToCart(lleg_joint_pos,lfoot_pose);

  ROS_INFO("Right foot x:%f, y:%f, z:%f",rfoot_pose.p.x(),rfoot_pose.p.y(),rfoot_pose.p.z());
  ROS_INFO(" Left foot x:%f, y:%f, z:%f",lfoot_pose.p.x(),lfoot_pose.p.y(),lfoot_pose.p.z());

  double freq = 200; // in HZ
  ros::Rate rate(freq);
  double time = 5; // in sec
  int numOfSteps = int (freq*time);

  double dz = (pelvis_des_pose.p.z()-pelvis_pose.p.z())/numOfSteps;

  //std::cout<<"pelvis_des_pose.p.z() = "<<pelvis_des_pose.p.z()<<std::endl;
  //std::cout<<"pelvis_pose.p.z() = "<<pelvis_pose.p.z()<<std::endl;
  //std::cout<<"dz = "<<dz<<std::endl;

  this->initializeROS();

  KDL::Frame pos = pelvis_pose;

  //Move CoM #1
  for(int i=0;i<numOfSteps;i++){
    pos.p.data[2] += dz;
    //std::cout<<"pos.p.z() = "<<pos.p.z()<<std::endl;

    this->movePelvis(pos, rleg_joint_pos_, "Right");
    this->movePelvis(pos, lleg_joint_pos_, "Left");

    this->publishMessageROS(rleg_joint_pos_, lleg_joint_pos_);
    this->setJointPosition(rleg_joint_pos_, lleg_joint_pos_);

    //pelvis_pose = pos;

    rate.sleep();
  }

  pelvis_pose = pos;


/*
  //this->get_joints_pos();

  //ROS_INFO("Test2!");
  //
  //while (getCallback){
  //  ros::spinOnce();
  //}
  //getCallback = true;
  //
  //KDL::Frame frm;
  //
  //rleg_fk_solver->JntToCart(rleg_joint_pos, frm);
  //
  //double fk_r = 0.0;
  //double fk_p = 0.0;
  //double fk_y = 0.0;
  //
  //frm.M.GetRPY(fk_r,fk_p,fk_y);
  //
  //KDL::Frame frm1 = KDL::Frame(KDL::Rotation::RPY(-fk_r, -fk_p, -fk_y),
  //                        KDL::Vector(pelvis_pose.p.x()-frm.p.x(),
  //                                    pelvis_pose.p.y()-frm.p.y()-y_offset,
  //                                    pelvis_pose.p.z()-frm.p.z())
  //                            );
  //
  //ROS_INFO("FK x: %f, y: %f, z: %f", frm.p.x(), frm.p.y(), frm.p.z());
  //ROS_INFO("Pelvis x: %f, y: %f, z: %f", frm1.p.x(), frm1.p.y(), frm1.p.z());

  //ROS_INFO("Test3!");
  //
  /////////////
  //
  //sensor_msgs::JointState legs_jnt_msg;
  //
  //legs_jnt_msg.name.resize(12);
  //legs_jnt_msg.position.resize(12);
  //
  //legs_jnt_msg.name[0]  = "r_hip_yaw"  ;
  //legs_jnt_msg.name[1]  = "r_hip_roll" ;
  //legs_jnt_msg.name[2]  = "r_hip_pitch";
  //legs_jnt_msg.name[3]  = "r_knee"     ;
  //legs_jnt_msg.name[4]  = "r_ank_pitch";
  //legs_jnt_msg.name[5]  = "r_ank_roll" ;
  //
  //legs_jnt_msg.name[6]  = "l_hip_yaw"  ;
  //legs_jnt_msg.name[7]  = "l_hip_roll" ;
  //legs_jnt_msg.name[8]  = "l_hip_pitch";
  //legs_jnt_msg.name[9]  = "l_knee"     ;
  //legs_jnt_msg.name[10] = "l_ank_pitch";
  //legs_jnt_msg.name[11] = "l_ank_roll" ;
  //
  //for(int i=0;i<12;i++){
  //  legs_jnt_msg.position[i] = 0.0;
  //  //legs_jnt_msg.velocity[i] = 10.0;
  //}
  //
  //ros::Publisher legs_joints_pub = node.advertise<sensor_msgs::JointState>("/robotis/direct_control/set_joint_states",10);
  //
  //legs_joints_pub.publish(legs_jnt_msg);



  //joint_msg leg_jnt_msg;
  //
  //ROS_INFO("Test4!");
  //
  //
  //leg_jnt_msg.name.resize(20);
  //leg_jnt_msg.angle.resize(20);
  //
  //leg_jnt_msg.name[0]   = "head_pan";
  //leg_jnt_msg.name[1]   = "head_tilt";
  //leg_jnt_msg.name[4]   = "l_el";
  //leg_jnt_msg.name[9]   = "l_sho_pitch";
  //leg_jnt_msg.name[10]  = "l_sho_roll";
  //leg_jnt_msg.name[13]  = "r_el";
  //leg_jnt_msg.name[18]  = "r_sho_pitch";
  //leg_jnt_msg.name[19]  = "r_sho_roll";
  //
  //
  //leg_jnt_msg.name[16]  = "r_hip_yaw";
  //leg_jnt_msg.name[15]  = "r_hip_roll";
  //leg_jnt_msg.name[14]  = "r_hip_pitch";
  //leg_jnt_msg.name[17]  = "r_knee";
  //leg_jnt_msg.name[11]  = "r_ank_pitch";
  //leg_jnt_msg.name[12]  = "r_ank_roll";
  //
  //leg_jnt_msg.name[7]   = "l_hip_yaw";
  //leg_jnt_msg.name[6]   = "l_hip_roll";
  //leg_jnt_msg.name[5]   = "l_hip_pitch";
  //leg_jnt_msg.name[8]   = "l_knee";
  //leg_jnt_msg.name[2]   = "l_ank_pitch";
  //leg_jnt_msg.name[3]   = "l_ank_roll";
  //
  //for(int i=0; i<20; i++){
  //  leg_jnt_msg.angle[i].data = 0.0;
  //}
  //
  ////for(int i=0;i<6;i++){
  ////  leg_jnt_msg.angle[i].data   = rleg_joint_pos(i) + 100*D2R;
  ////  leg_jnt_msg.angle[i+6].data = lleg_joint_pos(i) + 100*D2R;
  ////  ROS_INFO("Test5!");
  ////}
  ////
  ////for(int i=0;i<12;i++){
  ////  std::cout<<leg_jnt_msg.angle[i].data<<" ";
  ////}
  ////std::cout<<std::endl;
  //
  //legs_joints_msg.name.resize(20);
  //legs_joints_msg.position.resize(20);
  //
  //for(int i=0;i<20;i++){
  //  legs_joints_msg.name[i] = leg_jnt_msg.name[i];
  //  legs_joints_msg.position[i] = leg_jnt_msg.angle[i].data;
  //  ROS_INFO("Test6!");
  //}
  //
  //legs_joints_pub.publish(legs_joints_msg);

  ROS_INFO("Test7!");

*/


  this->deleteSolvers();
  this->deleteChains();

  //Move CoM #2

  ROS_INFO("Test1!");

  this->initialization(pelvis_pose);
  this->setJointPosition(rleg_joint_pos_,lleg_joint_pos_);

  this->rleg_fk_solver->JntToCart(rleg_joint_pos,rfoot_pose);
  this->lleg_fk_solver->JntToCart(lleg_joint_pos,lfoot_pose);

  ROS_INFO("Right foot x:%f, y:%f, z:%f",rfoot_pose.p.x(),rfoot_pose.p.y(),rfoot_pose.p.z());
  ROS_INFO(" Left foot x:%f, y:%f, z:%f",lfoot_pose.p.x(),lfoot_pose.p.y(),lfoot_pose.p.z());

  pelvis_des_pose = pelvis_pose;
  pelvis_des_pose.p.data[1]+= -y_offset;

  double dy = (pelvis_des_pose.p.y()-pelvis_pose.p.y())/numOfSteps;

  for(int i=0;i<numOfSteps;i++){
    pos.p.data[1] += dy;
    //std::cout<<"pos.p.z() = "<<pos.p.z()<<std::endl;

    this->movePelvis(pos, rleg_joint_pos_, "Right");
    this->movePelvis(pos, lleg_joint_pos_, "Left");

    this->publishMessageROS(rleg_joint_pos_, lleg_joint_pos_);
    this->setJointPosition(rleg_joint_pos_, lleg_joint_pos_);

    //pelvis_pose = pos;

    rate.sleep();
  }

  pelvis_pose = pos;

  ROS_INFO("Pelvis pose x:%f, y:%f, z:%f",pelvis_pose.p.x(),pelvis_pose.p.y(),pelvis_pose.p.z());

  this->deleteSolvers();
  this->deleteChains();


  ROS_INFO("Test2!");

  this->initialization(pelvis_pose);
  this->setJointPosition(rleg_joint_pos_,lleg_joint_pos_);

  this->rleg_fk_solver->JntToCart(rleg_joint_pos,rfoot_pose);
  this->lleg_fk_solver->JntToCart(lleg_joint_pos,lfoot_pose);

  ROS_INFO("Right foot x:%f, y:%f, z:%f",rfoot_pose.p.x(),rfoot_pose.p.y(),rfoot_pose.p.z());
  ROS_INFO(" Left foot x:%f, y:%f, z:%f",lfoot_pose.p.x(),lfoot_pose.p.y(),lfoot_pose.p.z());

  int n_step = int (freq*sp.step_duration);

  up_part = 0.25;
  down_part = 0.25;

  int n_up = int (up_part*n_step);
  int n_down = int (down_part*n_step);
  int n_mid = n_step - (n_up + n_down);

  std::vector<KDL::Frame> foot_poses(n_step);

  KDL::Frame cur_frm;

  delta_t_up = (M_PI/2)/n_up;
  delta_t_down = (M_PI/2)/n_down;

  for(int count=0;count<n_step;count++){

    if(count<=n_up){

      cur_frm = KDL::Frame(KDL::Rotation::RPY(0.0, 0.0, 0.0),
                           KDL::Vector(sp.step_length*up_part*(1-cos(delta_t_up*count))/1000,
                                       y_offset,
                                       sp.step_clearance*sin(delta_t_up*count)/1000
                                       )
                           );

      foot_poses.push_back(cur_frm);
      ROS_INFO("Step #%d x:%f, y:%f, z:%f",count,cur_frm.p.x(),cur_frm.p.y(),cur_frm.p.z());

    }

    if((count>n_up)&&(count<=n_up+n_mid)){

      cur_frm = KDL::Frame(KDL::Rotation::RPY(0.0, 0.0, 0.0),
                           KDL::Vector(sp.step_length/n_step*count/1000,
                                       y_offset,
                                       sp.step_clearance/1000
                                       )
                           );

      foot_poses.push_back(cur_frm);
      ROS_INFO("Step #%d x:%f, y:%f, z:%f",count,cur_frm.p.x(),cur_frm.p.y(),cur_frm.p.z());

    }

    if((count>n_up+n_mid)&&(count<=n_step)){

      cur_frm = KDL::Frame(KDL::Rotation::RPY(0.0, 0.0, 0.0),
                           KDL::Vector(sp.step_length*down_part*sin(delta_t_down*(count-(n_up+n_mid)))/1000 + sp.step_length/n_step*(n_up+n_mid)/1000,
                                       y_offset,
                                       sp.step_clearance*cos(delta_t_down*(count-(n_up+n_mid)))/1000
                                       )
                           );

      foot_poses.push_back(cur_frm);
      ROS_INFO("Step #%d x:%f, y:%f, z:%f",count,cur_frm.p.x(),cur_frm.p.y(),cur_frm.p.z());

    }

    //rate.sleep();
  }

}
