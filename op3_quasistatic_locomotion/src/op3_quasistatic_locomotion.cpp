#include "op3_quasistatic_locomotion.h"

op3_quasistatic_locomotion::op3_quasistatic_locomotion()
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

  getJointCallback = true;

  keyboard_run = false;
  keyboard_quit = false;

  manager_is_launched = false;
  rostopic_is_init = false;

}

op3_quasistatic_locomotion::~op3_quasistatic_locomotion(){

}

void op3_quasistatic_locomotion::deleteChains(){

  delete rleg_chain;
  delete lleg_chain;
}

void op3_quasistatic_locomotion::deleteSolvers(){

  delete rleg_fk_solver;
  delete rleg_ik_vel_solver;
  delete rleg_ik_pos_solver;

  delete lleg_fk_solver;
  delete lleg_ik_vel_solver;
  delete lleg_ik_pos_solver;

}

void op3_quasistatic_locomotion::initializeROS(){

  std::string module_name = "directcontrol";

  std::transform(module_name.begin(),module_name.end(),module_name.begin(), ::tolower);
  rostopic_is_init = false;

  if (module_name == "directcontrolmodule" || module_name == "directcontrol"){
    this->setModule("direct_control_module");
    leg_joints_pub = node.advertise<sensor_msgs::JointState>("/robotis/direct_control/set_joint_states", 0);
    rostopic_is_init = true;
  }
  else{
    if (module_name == "none"){
      this->setModule(module_name);
      leg_joints_pub = node.advertise<sensor_msgs::JointState>("/robotis/set_joint_states", 0);
      rostopic_is_init = true;
    }
    else{
      ROS_WARN("Wrong control module!");
      ROS_WARN("Exit...");
    }
  }

}

void op3_quasistatic_locomotion::managerJointPos(){
  present_joint_states_sub = node.subscribe("/robotis/present_joint_states",1000, &op3_quasistatic_locomotion::presStateCallback, this);
  //ROS_INFO("CALLBACK TEST 0!");

  ros::Rate rate = 1000;

  while(ros::ok() && getJointCallback){

    ros::spinOnce();
    rate.sleep();

  }

  getJointCallback = true;

}

void op3_quasistatic_locomotion::presStateCallback(const sensor_msgs::JointState::ConstPtr &jnt_data)
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

  getJointCallback = false;

  //ROS_INFO("CALLBACK TEST 1!");

  present_joint_states_sub.shutdown();

}

void op3_quasistatic_locomotion::publishMessageROS(Eigen::VectorXd rleg_jnt_angle_, Eigen::VectorXd lleg_jnt_angle_){

  sensor_msgs::JointState joint_angles_msg;

  //Right leg
  joint_angles_msg.name.push_back("r_hip_yaw");
  joint_angles_msg.position.push_back(rleg_jnt_angle_(0));

  joint_angles_msg.name.push_back("r_hip_roll");
  joint_angles_msg.position.push_back(rleg_jnt_angle_(1));

  joint_angles_msg.name.push_back("r_hip_pitch");
  joint_angles_msg.position.push_back(rleg_jnt_angle_(2));

  joint_angles_msg.name.push_back("r_knee");
  joint_angles_msg.position.push_back(rleg_jnt_angle_(3));

  joint_angles_msg.name.push_back("r_ank_pitch");
  joint_angles_msg.position.push_back(rleg_jnt_angle_(4));

  joint_angles_msg.name.push_back("r_ank_roll");
  joint_angles_msg.position.push_back(rleg_jnt_angle_(5));

  //Left leg
  joint_angles_msg.name.push_back("l_hip_yaw");
  joint_angles_msg.position.push_back(lleg_jnt_angle_(0));

  joint_angles_msg.name.push_back("l_hip_roll");
  joint_angles_msg.position.push_back(lleg_jnt_angle_(1));

  joint_angles_msg.name.push_back("l_hip_pitch");
  joint_angles_msg.position.push_back(lleg_jnt_angle_(2));

  joint_angles_msg.name.push_back("l_knee");
  joint_angles_msg.position.push_back(lleg_jnt_angle_(3));

  joint_angles_msg.name.push_back("l_ank_pitch");
  joint_angles_msg.position.push_back(lleg_jnt_angle_(4));

  joint_angles_msg.name.push_back("l_ank_roll");
  joint_angles_msg.position.push_back(lleg_jnt_angle_(5));



  leg_joints_pub.publish(joint_angles_msg);

}

void op3_quasistatic_locomotion::initialization(KDL::Frame pelvis_pose){

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

void op3_quasistatic_locomotion::setJointPosition(Eigen::VectorXd rleg_joint_position_, Eigen::VectorXd lleg_joint_position_){

  rleg_joint_pos.data = rleg_joint_position_;
  lleg_joint_pos.data = lleg_joint_position_;

}

void op3_quasistatic_locomotion::getJointPosition(Eigen::VectorXd &rleg_joint_position_, Eigen::VectorXd &lleg_joint_position_){

  rleg_joint_position_ = rleg_joint_pos.data;
  lleg_joint_position_ = lleg_joint_pos.data;

}

bool op3_quasistatic_locomotion::getFeetPose(){

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

bool op3_quasistatic_locomotion::moveFoot(KDL::Frame foot_des_pose, Eigen::VectorXd &leg_des_joint_pos_, std::string legType){

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
      //ROS_INFO("Right leg (deg) hip_yaw:%f, hip_r:%f, hip_p:%f, kn_p:%f, an_p :%f, an_r:%f",
      //         leg_des_joint_pos_(0)*R2D,leg_des_joint_pos_(1)*R2D,leg_des_joint_pos_(2)*R2D,
      //         leg_des_joint_pos_(3)*R2D,leg_des_joint_pos_(4)*R2D,leg_des_joint_pos_(5)*R2D
      //         );

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
        //ROS_INFO("Left leg (deg) hip_yaw:%f, hip_r:%f, hip_p:%f, kn_p:%f, an_p :%f, an_r:%f",
        //         leg_des_joint_pos_(0)*R2D,leg_des_joint_pos_(1)*R2D,leg_des_joint_pos_(2)*R2D,
        //         leg_des_joint_pos_(3)*R2D,leg_des_joint_pos_(4)*R2D,leg_des_joint_pos_(5)*R2D
        //         );

        return true;
      }
    }
    else{
      ROS_WARN("moveFoot: INCORRECT LEG INPUT");
      return false;
    }
  }

}

bool op3_quasistatic_locomotion::movePelvis(KDL::Frame pelvis_des_pose, Eigen::VectorXd &leg_des_joint_pos_, std::string legType){

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
      ROS_WARN("movePelvis: INCORRECT LEG INPUT");
      return false;
    }
  }

}

bool op3_quasistatic_locomotion::footTrajectoryGeneration(std::vector<KDL::Frame> &foot_poses, stepParam sp, std::string legType){

  int n_step = int (sp.freq*sp.step_duration);
  up_part = 0.25;
  down_part = 0.25; //up_part + down_part <= 1

  int n_up = int (up_part*n_step);
  int n_down = int (down_part*n_step);
  int n_mid = n_step - (n_up + n_down);

  double x_val = 0.0;
  double y_val = 0.0;

  std::transform(legType.begin(),legType.end(),legType.begin(), ::tolower);

  if(legType == "right"){
    y_val = -y_offset;
    x_val = rfoot_pose.p.x();
  }
  else{
    if (legType == "left"){
      y_val = y_offset;
      x_val = lfoot_pose.p.x();
    }
    else{
      ROS_WARN("footTrajectoryGeneration: INCORRECT LEG INPUT");
      return false;
    }
  }

  KDL::Frame cur_frm;

  double delta_t_up = (M_PI/2)/n_up;
  double delta_t_down = (M_PI/2)/n_down;

  for(int count=0;count<n_step;count++){

    if(count<=n_up){

      cur_frm = KDL::Frame(KDL::Rotation::RPY(0.0, 0.0, 0.0),
                           KDL::Vector(sp.step_length*up_part*(1-cos(delta_t_up*count))/1000 +x_val,
                                       y_val,
                                       sp.step_clearance*sin(delta_t_up*count)/1000
                                       )
                           );

      foot_poses.push_back(cur_frm);
      //ROS_INFO("Step #%d x:%f, y:%f, z:%f",count,cur_frm.p.x(),cur_frm.p.y(),cur_frm.p.z());

    }

    if((count>n_up)&&(count<=n_up+n_mid)){

      cur_frm = KDL::Frame(KDL::Rotation::RPY(0.0, 0.0, 0.0),
                           KDL::Vector(sp.step_length/n_step*count/1000 + x_val,
                                       y_val,
                                       sp.step_clearance/1000
                                       )
                           );

      foot_poses.push_back(cur_frm);
      //ROS_INFO("Step #%d x:%f, y:%f, z:%f",count,cur_frm.p.x(),cur_frm.p.y(),cur_frm.p.z());

    }

    if((count>n_up+n_mid)&&(count<=n_step)){

      cur_frm = KDL::Frame(KDL::Rotation::RPY(0.0, 0.0, 0.0),
                           KDL::Vector(sp.step_length*down_part*sin(delta_t_down*(count-(n_up+n_mid)))/1000 + sp.step_length/n_step*(n_up+n_mid)/1000 + x_val,
                                       y_val,
                                       sp.step_clearance*cos(delta_t_down*(count-(n_up+n_mid)))/1000
                                       )
                           );

      foot_poses.push_back(cur_frm);
      //ROS_INFO("Step #%d x:%f, y:%f, z:%f",count,cur_frm.p.x(),cur_frm.p.y(),cur_frm.p.z());

    }


  }

  return true;

}

bool op3_quasistatic_locomotion::launchManager(){

  std::system(RUNMNGRSCRIPT);

  manager_is_launched = true;

  ros::Duration(7.5).sleep();

  return true; // TODO: error processing!

}

void op3_quasistatic_locomotion::goToInitialPose(KDL::Frame pelvis_des_pose, stepParam sp){

  ROS_INFO("Initial position trajectory planning");

  pelvis_pose = KDL::Frame(
                    KDL::Rotation::RPY(0.0,0.0,0.0),
                    KDL::Vector(0.0,0.0,0.3697) // z - max height
                       );

  // Fake initialization just to get Pelvis pose
  this->initialization(pelvis_pose);

  Eigen::VectorXd rleg_joint_pos_;
  Eigen::VectorXd lleg_joint_pos_;
  rleg_joint_pos_.resize(JOINT_NUM);
  lleg_joint_pos_.resize(JOINT_NUM);

  //ROS_INFO("Test1");
  this->managerJointPos();

  this->initializeROS();
  if (!rostopic_is_init)
    return;

  if (!this->getFeetPose()){
    return;
  }
  //Get foot orrientation
  double roll,pitch,yaw;
  rfoot_pose.M.GetRPY(roll,pitch,yaw);

  // Pelvis pose after op3_manager launch
  pelvis_pose = KDL::Frame(KDL::Rotation::RPY(0.0, 0.0, 0.0),
                           KDL::Vector(-rfoot_pose.p.x(), 0.0, 0.3697-rfoot_pose.p.z())
                           );

  ROS_INFO("Pelvis position x:%f, y:%f, z:%f",pelvis_pose.p.x(),pelvis_pose.p.y(),pelvis_pose.p.z());
  ROS_INFO("Pelvis orientation r:%f, p:%f, y:%f", 0.0, 0.0, 0.0);

  this->deleteSolvers();
  this->deleteChains();

  //ROS_INFO("Test3");

  // true initialization
  this->initialization(pelvis_pose);
  if (!this->getFeetPose()){
    return;
  }
  ROS_INFO("Right foot x:%f, y:%f, z:%f",rfoot_pose.p.x(),rfoot_pose.p.y(),rfoot_pose.p.z());
  ROS_INFO(" Left foot x:%f, y:%f, z:%f",lfoot_pose.p.x(),lfoot_pose.p.y(),lfoot_pose.p.z());

  //ROS_INFO("Test4");

  double time = 5; // in sec
  int numOfSteps = int (sp.freq*time);

  double des_roll, des_pitch, des_yaw;
  pelvis_des_pose.M.GetRPY(des_roll,des_pitch,des_yaw);

  double dx = (pelvis_des_pose.p.x()-pelvis_pose.p.x())/numOfSteps;
  double dz = (pelvis_des_pose.p.z()-pelvis_pose.p.z())/numOfSteps;
  double dp = (des_pitch-pitch)/numOfSteps; // (des_pitch-pitch)/numOfSteps;

  KDL::Frame pos = pelvis_pose;

  double x = pos.p.data[0];
  double z = pos.p.data[2];

  for(int i=0;i<numOfSteps;i++){
    x += dx;
    z += dz;
    pitch += dp;

    pos = KDL::Frame(KDL::Rotation::RPY(0.0, pitch, 0.0),
                     KDL::Vector(x, 0.0, z));

    //std::cout<<"pos.p.x() = "<<pos.p.x()<<std::endl;
    //std::cout<<"pos.p.z() = "<<pos.p.z()<<std::endl;
    //std::cout<<"pos_pitch = "<<pitch<<std::endl;

    this->movePelvis(pos, rleg_joint_pos_, "Right");
    this->movePelvis(pos, lleg_joint_pos_, "Left");

    rleg_joint_angles.push_back(rleg_joint_pos_);
    lleg_joint_angles.push_back(lleg_joint_pos_);

    this->setJointPosition(rleg_joint_pos_, lleg_joint_pos_);

  }

  pelvis_pose = pos;

  ROS_INFO("Test5");

  this->deleteSolvers();
  this->deleteChains();

  ROS_INFO("Test6");


}

void op3_quasistatic_locomotion::initCoMTranslation(std::string legType, stepParam sp){

  //ROS_INFO("Test7");

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
      ROS_WARN("initCoMTranslation: INCORRECT LEG INPUT");
      return;
    }
  }

  ROS_INFO("Initial CoM translation planning");

  this->initialization(pelvis_pose);

  if (!this->getFeetPose()){
    return;
  }

  ROS_INFO("Right foot x:%f, y:%f, z:%f",rfoot_pose.p.x(),rfoot_pose.p.y(),rfoot_pose.p.z());
  ROS_INFO(" Left foot x:%f, y:%f, z:%f",lfoot_pose.p.x(),lfoot_pose.p.y(),lfoot_pose.p.z());

  Eigen::VectorXd rleg_joint_pos_;
  Eigen::VectorXd lleg_joint_pos_;

  this->getJointPosition(rleg_joint_pos_, lleg_joint_pos_);

  rleg_joint_pos_.resize(JOINT_NUM);
  lleg_joint_pos_.resize(JOINT_NUM);

  KDL::Frame pos = pelvis_pose;
  KDL::Frame pelvis_des_pose = pelvis_pose;
  pelvis_des_pose.p.data[1]+= y_val;

  //ros::Rate rate(sp.freq);
  double transl_time = 2.0; // sec
  int numOfSteps = int (transl_time*sp.freq);

  double dy = (pelvis_des_pose.p.y()-pelvis_pose.p.y())/numOfSteps;

  for(int i=0;i<numOfSteps;i++){
    pos.p.data[1] += dy;

    this->movePelvis(pos, rleg_joint_pos_, "Right");
    this->movePelvis(pos, lleg_joint_pos_, "Left");

    rleg_joint_angles.push_back(rleg_joint_pos_);
    lleg_joint_angles.push_back(lleg_joint_pos_);

    this->setJointPosition(rleg_joint_pos_, lleg_joint_pos_);

  }

  pelvis_pose = pos;

  ROS_INFO("Pelvis pose x:%f, y:%f, z:%f",pelvis_pose.p.x(),pelvis_pose.p.y(),pelvis_pose.p.z());

  this->deleteSolvers();
  this->deleteChains();

  //ROS_INFO("Test8");

}

void op3_quasistatic_locomotion::footTranslation(stepParam sp, std::string legType){

  ROS_INFO("Foot translation planning");

  std::transform(legType.begin(),legType.end(),legType.begin(), ::tolower);

  if((legType!="right")&&(legType!="left")){
    ROS_WARN("footTranslation: INCORRECT LEG INPUT");
    return;
  }

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

  //ros::Rate rate(sp.freq);
  int n_step = int (sp.freq*sp.step_duration);
  std::vector<KDL::Frame> foot_poses;

  this->footTrajectoryGeneration(foot_poses, sp, legType);

  for(int i=0; i<n_step; i++){

    if (legType == "right")
      this->moveFoot(foot_poses.at(i), rleg_joint_pos_, legType);
    if (legType == "left")
      this->moveFoot(foot_poses.at(i), lleg_joint_pos_, legType);

    rleg_joint_angles.push_back(rleg_joint_pos_);
    lleg_joint_angles.push_back(lleg_joint_pos_);

    this->setJointPosition(rleg_joint_pos_, lleg_joint_pos_);

  }

}

void op3_quasistatic_locomotion::translateCoM(std::string legType, stepParam sp){

  Eigen::VectorXd rleg_joint_pos_;
  Eigen::VectorXd lleg_joint_pos_;

  rleg_joint_pos_.resize(JOINT_NUM);
  lleg_joint_pos_.resize(JOINT_NUM);

  //get foot pose from FK
  KDL::Frame des_pose;

  if(!this->getFeetPose()){
    return;
  }

  ROS_INFO("Right foot x:%f, y:%f, z:%f",rfoot_pose.p.x(),rfoot_pose.p.y(),rfoot_pose.p.z());
  ROS_INFO(" Left foot x:%f, y:%f, z:%f",lfoot_pose.p.x(),lfoot_pose.p.y(),lfoot_pose.p.z());

  std::transform(legType.begin(),legType.end(),legType.begin(), ::tolower);

  if (legType == "right"){
    des_pose = pelvis_pose;                     // it actually doesn't matter, use for convinience
    des_pose.p.data[0] = rfoot_pose.p.data[0];
    des_pose.p.data[1] = rfoot_pose.p.data[1];
  }
  else{
    if (legType == "left"){
      des_pose = pelvis_pose;
      des_pose.p.data[0] = lfoot_pose.p.data[0];
      des_pose.p.data[1] = lfoot_pose.p.data[1];
    }
    else{
      ROS_WARN("translateCoM: INCORRECT LEG INPUT");
      return;
    }
  }

  KDL::Frame pos = pelvis_pose;

  double transl_time = 3.0; // 5 sec
  int numOfSteps = int (transl_time*sp.freq);

  double dx = (des_pose.p.x()-pelvis_pose.p.x())/numOfSteps;
  double dy = (des_pose.p.y()-pelvis_pose.p.y())/numOfSteps;

  for(int i=0;i<numOfSteps;i++){
    pos.p.data[0] += dx;
    pos.p.data[1] += dy;

    this->movePelvis(pos, rleg_joint_pos_, "Right");
    this->movePelvis(pos, lleg_joint_pos_, "Left");

    rleg_joint_angles.push_back(rleg_joint_pos_);
    lleg_joint_angles.push_back(lleg_joint_pos_);

    this->setJointPosition(rleg_joint_pos_, lleg_joint_pos_);

  }

  pelvis_pose = pos;

  ROS_INFO("Pelvis pose x:%f, y:%f, z:%f",pelvis_pose.p.x(),pelvis_pose.p.y(),pelvis_pose.p.z());

  this->deleteSolvers();
  this->deleteChains();

}

void op3_quasistatic_locomotion::quasiStaticPlaner(KDL::Frame pelvis_des_pose, stepParam sp){

  if((sp.num_of_steps <= 0.0)||(int(sp.num_of_steps)-sp.num_of_steps !=0)){
    ROS_WARN("quasiStatic: INCORRECT NUMBER OF STEPS");
    return;
  }

  //Set initial Pose
  this->goToInitialPose(pelvis_des_pose, sp);

  if (!rostopic_is_init)
    return;

  std::string init_leg = sp.init_leg;

  std::transform(init_leg.begin(),init_leg.end(),init_leg.begin(), ::tolower);

  std::string sup_leg;

  //Inital CoM translation
  if (init_leg == "right"){
    sup_leg = "left";
    this->initCoMTranslation(sup_leg, sp);
  }
  else{
    if (init_leg == "left"){
      sup_leg = "right";
      this->initCoMTranslation(sup_leg, sp);
    }
    else{
      ROS_WARN("quasiStatic: INCORRECT LEG INPUT");
      return;
    }
  }

  stepParam init_step = sp;
  init_step.step_duration /= 2.0;
  init_step.step_length /= 2.0;

  //First step
  this->footTranslation(init_step, init_leg);
  this->translateCoM(init_leg, sp);

  //Walking loop
  for(int i=2; i<=int(sp.num_of_steps); i++){
    ROS_INFO("Current step: %d", i);
    if (i%2 == 0){
      this->footTranslation(sp, sup_leg);
      this->translateCoM(sup_leg, sp);
    }
    if (i%2 == 1){
      this->footTranslation(sp, init_leg);
      this->translateCoM(init_leg, sp);
    }
  }

  ROS_INFO("Size of RLEG_JNT_ANGLES: %lu", rleg_joint_angles.size());
  ROS_INFO("Size of LLEG_JNT_ANGLES: %lu", lleg_joint_angles.size());

}

void op3_quasistatic_locomotion::locomotion(stepParam sp){

  //ROS_INFO("Test9");

  if(!rostopic_is_init)
    return;

  ros::Subscriber control_sub = node.subscribe("/op3_keyboard_control",
                                               100, &op3_quasistatic_locomotion::keyboardContolCallback,this);

  long unsigned int counter = 0;
  ros::Rate rate(sp.freq);
  while((ros::ok())&&(counter<rleg_joint_angles.size())&&(!keyboard_quit)){ // &&(!keyboard_quit)

    this->perFrameLocomotion(counter);

    if(keyboard_run) // keyboard_run
      counter++;

    ros::spinOnce();
    rate.sleep();
  }

}

void op3_quasistatic_locomotion::keyboardContolCallback(const std_msgs::String::ConstPtr &cntrl_status){

  if(cntrl_status->data == "run")
    keyboard_run = true;
  else
    keyboard_run = false;

  if(cntrl_status->data == "quit"){
    keyboard_quit = true;
  }

}

void op3_quasistatic_locomotion::perFrameLocomotion(unsigned long int num_of_frame){

  //ROS_INFO("Test10");

  if(ros::ok())
    this->publishMessageROS(rleg_joint_angles.at(num_of_frame), lleg_joint_angles.at(num_of_frame));

}

void op3_quasistatic_locomotion::getAnglesVectors(std::vector<Eigen::VectorXd> &rleg_joint_angles_,
                                           std::vector<Eigen::VectorXd> &lleg_joint_angles_)
{
  for (unsigned long int cntr = 0; cntr < rleg_joint_angles.size(); cntr++){
   rleg_joint_angles_.push_back(rleg_joint_angles.at(cntr));
   lleg_joint_angles_.push_back(lleg_joint_angles.at(cntr));
  }

}

bool op3_quasistatic_locomotion::getCurrentModule(std::vector<std::string> &joint_name, std::vector<std::string> &joint_module){

  ros::ServiceClient get_joint_module_client = node.serviceClient<robotis_controller_msgs::GetJointModule>("robotis/get_present_joint_ctrl_modules");

  robotis_controller_msgs::GetJointModule srv_msg;
  srv_msg.request.joint_name = joint_name;

  if(get_joint_module_client.call(srv_msg)){
    for(unsigned long int i=0; i<joint_name.size(); i++){
      ROS_INFO("%s: %s", srv_msg.response.joint_name.at(i).c_str(), srv_msg.response.module_name.at(i).c_str());
      joint_module.at(i) = srv_msg.response.module_name.at(i);
    }
  }

  return true;

}

void op3_quasistatic_locomotion::setJointModule(const std::vector<std::string> &joint_name, const std::vector<std::string> &module_name){

  ros::ServiceClient set_joint_module_client = node.serviceClient<robotis_controller_msgs::SetJointModule>("/robotis/set_present_joint_ctrl_modules");

  robotis_controller_msgs::SetJointModule set_mod_srv;
  set_mod_srv.request.joint_name = joint_name;
  set_mod_srv.request.module_name = module_name;

  if (set_joint_module_client.call(set_mod_srv) == false){
    ROS_WARN("Failed to set module");
    return;
  }

}

void op3_quasistatic_locomotion::setLegsModule(std::string moduleName){

  //std::transform(moduleName.begin(),moduleName.end(),moduleName.begin(), ::tolower);
  //
  //std::string module;
  //
  //if((moduleName == "directcontrolmodule")||(moduleName == "directcontrol"))
  //  module = "direct_control_module";
  //else{
  //  if(moduleName == "none")
  //    module = "none";
  //  else{
  //    ROS_WARN("Wrong control module!");
  //    ROS_WARN("Exit...");
  //    return false;
  //  }
  //}

  std::string module = moduleName;

  std::vector<std::string> joint_name;

  joint_name.push_back("r_hip_yaw");
  joint_name.push_back("r_hip_roll");
  joint_name.push_back("r_hip_pitch");
  joint_name.push_back("r_knee");
  joint_name.push_back("r_ank_pitch");
  joint_name.push_back("r_ank_roll");

  joint_name.push_back("l_hip_yaw");
  joint_name.push_back("l_hip_roll");
  joint_name.push_back("l_hip_pitch");
  joint_name.push_back("l_knee");
  joint_name.push_back("l_ank_pitch");
  joint_name.push_back("l_ank_roll");

  std::vector<std::string> joint_module;
  joint_module.resize(2*JOINT_NUM);

  this->getCurrentModule(joint_name, joint_module);

  for (std::vector<std::string>::iterator i = joint_module.begin();
       i != joint_module.end(); i++){
    if(*i != module) // direct_control_module
      *i = module;
  }
  this->setJointModule(joint_name, joint_module);
  ROS_INFO(" ");
  this->getCurrentModule(joint_name, joint_module);

  //return true;
}

void op3_quasistatic_locomotion::setModule(std::string moduleName){

  ros::ServiceClient set_joint_module_client = node.serviceClient<robotis_controller_msgs::SetModule>("/robotis/set_present_ctrl_modules");

  robotis_controller_msgs::SetModule set_module_srv;
  set_module_srv.request.module_name = moduleName;

  if (set_joint_module_client.call(set_module_srv) == false)
  {
    ROS_WARN("Failed to set module");
    return;
  }

}
