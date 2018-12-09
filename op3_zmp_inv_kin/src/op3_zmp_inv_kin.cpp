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
  freq = 200; // in HZ

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

void op3_zmp_inv_kin::getJointPosition(Eigen::VectorXd &rleg_joint_position_, Eigen::VectorXd &lleg_joint_position_){

  rleg_joint_position_ = rleg_joint_pos.data;
  lleg_joint_position_ = lleg_joint_pos.data;

}

bool op3_zmp_inv_kin::getFeetPose(){

  int fk_error = rleg_fk_solver->JntToCart(rleg_joint_pos,rfoot_pose);

  if (fk_error < 0){
    ROS_WARN("RIGHT LEG FK ERROR");
    return false;
  }

  fk_error = lleg_fk_solver->JntToCart(lleg_joint_pos,lfoot_pose);
  if (fk_error < 0){
    ROS_WARN("LEFT LEG FK ERROR");
    return false;
  }

  return true;

}

bool op3_zmp_inv_kin::moveFoot(KDL::Frame foot_des_pose, Eigen::VectorXd &leg_des_joint_pos_, std::string legType){

  std::transform(legType.begin(),legType.end(),legType.begin(), ::tolower);
  int ik_error = 0;

  if (legType == "right"){

    KDL::JntArray rleg_des_joint_pos;
    ik_error = rleg_ik_pos_solver->CartToJnt(rleg_joint_pos, foot_des_pose, rleg_des_joint_pos);

    if (ik_error != 0)
    {
      ROS_WARN("RIGHT LEG IK ERROR : %s", rleg_ik_pos_solver->strError(ik_error));

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

      return true;
    }

  }
  else{
    if (legType == "left"){

      KDL::JntArray lleg_des_joint_pos;
      ik_error = lleg_ik_pos_solver->CartToJnt(lleg_joint_pos, foot_des_pose, lleg_des_joint_pos);

      if (ik_error != 0)
      {
        ROS_WARN("LEFT LEG IK ERROR : %s", lleg_ik_pos_solver->strError(ik_error));

        return false;
      }
      else{
        for (int i=0; i<JOINT_NUM;i++){
            leg_des_joint_pos_(i) = lleg_des_joint_pos(i);
        }
        ROS_INFO("Left leg (deg) hip_yaw:%f, hip_r:%f, hip_p:%f, kn_p:%f, an_p :%f, an_r:%f",
                 leg_des_joint_pos_(0)*R2D,leg_des_joint_pos_(1)*R2D,leg_des_joint_pos_(2)*R2D,
                 leg_des_joint_pos_(3)*R2D,leg_des_joint_pos_(4)*R2D,leg_des_joint_pos_(5)*R2D
                 );

        return true;
      }
    }
    else{
      ROS_INFO("INCORRECT INPUT FOR LEG TYPE");
      return false;
    }
  }

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

    KDL::JntArray rleg_des_joint_pos;
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

      return true;
    }
  }
  else{
    if(legType == "left"){

      KDL::JntArray lleg_des_joint_pos;
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

        return true;
      }
    }
    else{
      ROS_WARN("INCORRECT INPUT FOR LEG TYPE");
      return false;
    }
  }

  //std::cout<<"pseudo_z = "<<pseudo.p.z()<<std::endl;

}

bool op3_zmp_inv_kin::footTrajectoryGeneration(std::vector<KDL::Frame> &foot_poses, stepParam sp, std::string legType){

  int n_step = int (freq*sp.step_duration);
  up_part = 0.25;
  down_part = 0.25; //up_part + down_part <= 1

  int n_up = int (up_part*n_step);
  int n_down = int (down_part*n_step);
  int n_mid = n_step - (n_up + n_down);

  double y_val = 0.0;

  std::transform(legType.begin(),legType.end(),legType.begin(), ::tolower);

  if(legType == "right"){
    y_val = -y_offset;
  }
  else{
    if (legType == "left"){
      y_val = y_offset;
    }
    else{
      ROS_INFO("INCORRECT INPUT FOR LEG TYPE");
      return false;
    }
  }

  KDL::Frame cur_frm;

  double delta_t_up = (M_PI/2)/n_up;
  double delta_t_down = (M_PI/2)/n_down;

  for(int count=0;count<n_step;count++){

    if(count<=n_up){

      cur_frm = KDL::Frame(KDL::Rotation::RPY(0.0, 0.0, 0.0),
                           KDL::Vector(sp.step_length*up_part*(1-cos(delta_t_up*count))/1000,
                                       y_val,
                                       sp.step_clearance*sin(delta_t_up*count)/1000
                                       )
                           );

      foot_poses.push_back(cur_frm);
      //ROS_INFO("Step #%d x:%f, y:%f, z:%f",count,cur_frm.p.x(),cur_frm.p.y(),cur_frm.p.z());

    }

    if((count>n_up)&&(count<=n_up+n_mid)){

      cur_frm = KDL::Frame(KDL::Rotation::RPY(0.0, 0.0, 0.0),
                           KDL::Vector(sp.step_length/n_step*count/1000,
                                       y_val,
                                       sp.step_clearance/1000
                                       )
                           );

      foot_poses.push_back(cur_frm);
      //ROS_INFO("Step #%d x:%f, y:%f, z:%f",count,cur_frm.p.x(),cur_frm.p.y(),cur_frm.p.z());

    }

    if((count>n_up+n_mid)&&(count<=n_step)){

      cur_frm = KDL::Frame(KDL::Rotation::RPY(0.0, 0.0, 0.0),
                           KDL::Vector(sp.step_length*down_part*sin(delta_t_down*(count-(n_up+n_mid)))/1000 + sp.step_length/n_step*(n_up+n_mid)/1000,
                                       y_val,
                                       sp.step_clearance*cos(delta_t_down*(count-(n_up+n_mid)))/1000
                                       )
                           );

      foot_poses.push_back(cur_frm);
      //ROS_INFO("Step #%d x:%f, y:%f, z:%f",count,cur_frm.p.x(),cur_frm.p.y(),cur_frm.p.z());

    }

    ROS_INFO("Step #%d x:%f, y:%f, z:%f",count,foot_poses.at(count).p.x(),foot_poses.at(count).p.y(),foot_poses.at(count).p.z());

  }

  return true;

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

void op3_zmp_inv_kin::goToInitialPose(KDL::Frame pelvis_des_pose){

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

  if (!this->getFeetPose()){
    return;
  }

  ROS_INFO("Right foot x:%f, y:%f, z:%f",rfoot_pose.p.x(),rfoot_pose.p.y(),rfoot_pose.p.z());
  ROS_INFO(" Left foot x:%f, y:%f, z:%f",lfoot_pose.p.x(),lfoot_pose.p.y(),lfoot_pose.p.z());

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

  this->deleteSolvers();
  this->deleteChains();

}

void op3_zmp_inv_kin::initCoMTranslation(std::string legType){

  double y_val = 0.0;

  std::transform(legType.begin(),legType.end(),legType.begin(), ::tolower);

  if(legType == "right"){
    y_val = -y_offset;
  }
  else{
    if (legType == "left"){
      y_val = y_offset;
    }
    else{
      ROS_INFO("INCORRECT INPUT FOR LEG TYPE");
      return;
    }
  }

  ROS_INFO("Test1!");

  this->initialization(pelvis_pose);

  if (!this->getFeetPose()){
    return;
  }

  Eigen::VectorXd rleg_joint_pos_;
  Eigen::VectorXd lleg_joint_pos_;

  this->getJointPosition(rleg_joint_pos_, lleg_joint_pos_);

  rleg_joint_pos_.resize(JOINT_NUM);
  lleg_joint_pos_.resize(JOINT_NUM);

  ROS_INFO("Right foot x:%f, y:%f, z:%f",rfoot_pose.p.x(),rfoot_pose.p.y(),rfoot_pose.p.z());
  ROS_INFO(" Left foot x:%f, y:%f, z:%f",lfoot_pose.p.x(),lfoot_pose.p.y(),lfoot_pose.p.z());

  KDL::Frame pos = pelvis_pose;
  KDL::Frame pelvis_des_pose = pelvis_pose;
  pelvis_des_pose.p.data[1]+= y_val;

  ros::Rate rate(freq);
  double transl_time = 2.0; // sec
  int numOfSteps = int (transl_time*freq);

  double dy = (pelvis_des_pose.p.y()-pelvis_pose.p.y())/numOfSteps;

  for(int i=0;i<numOfSteps;i++){
    pos.p.data[1] += dy;

    this->movePelvis(pos, rleg_joint_pos_, "Right");
    this->movePelvis(pos, lleg_joint_pos_, "Left");

    this->publishMessageROS(rleg_joint_pos_, lleg_joint_pos_);
    this->setJointPosition(rleg_joint_pos_, lleg_joint_pos_);

    rate.sleep();
  }

  pelvis_pose = pos;

  ROS_INFO("Pelvis pose x:%f, y:%f, z:%f",pelvis_pose.p.x(),pelvis_pose.p.y(),pelvis_pose.p.z());

  this->deleteSolvers();
  this->deleteChains();

}

void op3_zmp_inv_kin::footTranslation(stepParam sp, std::string legType){

  ROS_INFO("Test2!");

  //std::transform(legType.begin(),legType.end(),legType.begin(), ::tolower);

  this->initialization(pelvis_pose);

  Eigen::VectorXd rleg_joint_pos_;
  Eigen::VectorXd lleg_joint_pos_;

  rleg_joint_pos_.resize(JOINT_NUM);
  lleg_joint_pos_.resize(JOINT_NUM);

  this->getJointPosition(rleg_joint_pos_, lleg_joint_pos_);

  if (!this->getFeetPose()){
    return;
  }

  ROS_INFO("Right foot x:%f, y:%f, z:%f",rfoot_pose.p.x(),rfoot_pose.p.y(),rfoot_pose.p.z());
  ROS_INFO(" Left foot x:%f, y:%f, z:%f",lfoot_pose.p.x(),lfoot_pose.p.y(),lfoot_pose.p.z());

  ros::Rate rate(freq);
  int n_step = int (freq*sp.step_duration);
  std::vector<KDL::Frame> foot_poses;

  this->footTrajectoryGeneration(foot_poses, sp, legType);

  for(int i=0; i<n_step; i++){

    this->moveFoot(foot_poses.at(i), lleg_joint_pos_, legType);

    this->publishMessageROS(rleg_joint_pos_, lleg_joint_pos_);
    this->setJointPosition(rleg_joint_pos_, lleg_joint_pos_);

    rate.sleep();

  }

}

void op3_zmp_inv_kin::quasiStatic(KDL::Frame pelvis_des_pose, stepParam sp){

  this->goToInitialPose(pelvis_des_pose);

  std::string init_leg = sp.init_leg;

  std::transform(init_leg.begin(),init_leg.end(),init_leg.begin(), ::tolower);

  if (init_leg == "right"){
    this->initCoMTranslation("left");
  }
  else{
    if (init_leg == "left"){
      this->initCoMTranslation("right");
    }
    else{
      ROS_WARN("INCORRECT INPUT");
      return;
    }
  }

  this->footTranslation(sp, init_leg);


}




