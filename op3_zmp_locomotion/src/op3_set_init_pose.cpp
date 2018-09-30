#include "op3_zmp_locomotion.h"

#define PELVIS_INIT_POSITION (0.3697)

bool op3_zmp_locomotion::setInitPose(KDL::Frame pelvis_des_pose, Eigen::VectorXd &rleg_cur_joint_pos_, Eigen::VectorXd &lleg_cur_joint_pos_){

  //ros::init(argc,argv,"op3_move_pelvis");
  //ros::NodeHandle node;

/*
//Right leg
  ros::Publisher r_hip_y_pub = node.advertise<std_msgs::Float64>("robotis_op3/r_hip_yaw_position/command",20);
  ros::Publisher r_hip_r_pub = node.advertise<std_msgs::Float64>("robotis_op3/r_hip_roll_position/command",20);
  ros::Publisher r_hip_p_pub = node.advertise<std_msgs::Float64>("robotis_op3/r_hip_pitch_position/command",20);
  ros::Publisher r_kn_p_pub = node.advertise<std_msgs::Float64>("robotis_op3/r_knee_position/command",20);
  ros::Publisher r_an_p_pub = node.advertise<std_msgs::Float64>("robotis_op3/r_ank_pitch_position/command",20);
  ros::Publisher r_an_r_pub = node.advertise<std_msgs::Float64>("robotis_op3/r_ank_roll_position/command",20);
//Left leg
  ros::Publisher l_hip_y_pub = node.advertise<std_msgs::Float64>("robotis_op3/l_hip_yaw_position/command",20);
  ros::Publisher l_hip_r_pub = node.advertise<std_msgs::Float64>("robotis_op3/l_hip_roll_position/command",20);
  ros::Publisher l_hip_p_pub = node.advertise<std_msgs::Float64>("robotis_op3/l_hip_pitch_position/command",20);
  ros::Publisher l_kn_p_pub = node.advertise<std_msgs::Float64>("robotis_op3/l_knee_position/command",20);
  ros::Publisher l_an_p_pub = node.advertise<std_msgs::Float64>("robotis_op3/l_ank_pitch_position/command",20);
  ros::Publisher l_an_r_pub = node.advertise<std_msgs::Float64>("robotis_op3/l_ank_roll_position/command",20);
*/

  ros::Rate loop_rate(20);

  //op3_zmp_locomotion move_pelvis;

  this->initialize(pelvis_des_pose,
                    KDL::Frame(KDL::Vector(0.0 , -0.035, 0.0)),
                    KDL::Frame(KDL::Vector(0.0 , 0.035, 0.0)));

  double pelvis_roll;
  double pelvis_pitch;
  double pelvis_yaw;

  pelvis_des_pose.M.GetRPY(pelvis_roll,pelvis_pitch,pelvis_yaw);

  Eigen::VectorXd rleg_joint_pos_, lleg_joint_pos_;

  rleg_joint_pos_.resize(JOINT_NUM);
  rleg_joint_pos_(0) = 0.001;
  rleg_joint_pos_(1) = 0.001;
  rleg_joint_pos_(2) = 0.001;
  rleg_joint_pos_(3) = 0.001;
  rleg_joint_pos_(4) = 0.001;
  rleg_joint_pos_(5) = 0.001;

  lleg_joint_pos_.resize(JOINT_NUM);
  lleg_joint_pos_(0) = 0.001;
  lleg_joint_pos_(1) = 0.001;
  lleg_joint_pos_(2) = 0.001;
  lleg_joint_pos_(3) = 0.001;
  lleg_joint_pos_(4) = 0.001;
  lleg_joint_pos_(5) = 0.001;

  this->setJointPosition(rleg_joint_pos_,lleg_joint_pos_);

  Eigen::VectorXd rleg_des_joint_pos_;
  Eigen::VectorXd lleg_des_joint_pos_;

  double pelvis_current_position = PELVIS_INIT_POSITION;
  double pelvis_target_position_z = pelvis_des_pose.p.z();
  double step = 0.0002;

  ROS_INFO("SetInitPose is run!");

/*
//Right leg
  std_msgs::Float64 r_an_r_msg;
  std_msgs::Float64 r_an_p_msg;
  std_msgs::Float64 r_kn_p_msg;
  std_msgs::Float64 r_hip_p_msg;
  std_msgs::Float64 r_hip_r_msg;
  std_msgs::Float64 r_hip_y_msg;
//Left leg
  std_msgs::Float64 l_an_r_msg;
  std_msgs::Float64 l_an_p_msg;
  std_msgs::Float64 l_kn_p_msg;
  std_msgs::Float64 l_hip_p_msg;
  std_msgs::Float64 l_hip_r_msg;
  std_msgs::Float64 l_hip_y_msg;
  */

  while (pelvis_current_position > pelvis_target_position_z){

    pelvis_current_position -= step;

    pelvis_current_pose = KDL::Frame(KDL::Rotation::RPY(pelvis_roll, pelvis_pitch, pelvis_yaw), KDL::Vector(0.0, 0.0, pelvis_current_position));

    this->moveCOMToRightLeg(KDL::Frame(pelvis_des_pose.M,
                            //KDL::Frame(KDL::Rotation::RPY(pelvis_roll, pelvis_pitch, pelvis_yaw),
                                    KDL::Vector(0.0, 0.0, pelvis_current_position)),
                                    rleg_des_joint_pos_);

    this->moveCOMToLeftLeg(KDL::Frame(pelvis_des_pose.M,
                           //KDL::Frame(KDL::Rotation::RPY(pelvis_roll, pelvis_pitch, pelvis_yaw),
                                    KDL::Vector(0.0, 0.0, pelvis_current_position)),
                                    lleg_des_joint_pos_);

  //Right leg
    r_an_r_msg.data = rleg_des_joint_pos_(0);
    r_an_p_msg.data = -rleg_des_joint_pos_(1);
    r_kn_p_msg.data = -rleg_des_joint_pos_(2);
    r_hip_p_msg.data = -rleg_des_joint_pos_(3);
    r_hip_r_msg.data = rleg_des_joint_pos_(4);
    r_hip_y_msg.data = rleg_des_joint_pos_(5);
  //Left leg
    l_an_r_msg.data = lleg_des_joint_pos_(0);
    l_an_p_msg.data = lleg_des_joint_pos_(1);
    l_kn_p_msg.data = lleg_des_joint_pos_(2);
    l_hip_p_msg.data = lleg_des_joint_pos_(3);
    l_hip_r_msg.data = lleg_des_joint_pos_(4);
    l_hip_y_msg.data = lleg_des_joint_pos_(5);
  //From pelvis to ankles
    r_hip_y_pub.publish(r_hip_y_msg);
    l_hip_y_pub.publish(l_hip_y_msg);

    r_hip_r_pub.publish(r_hip_r_msg);
    l_hip_r_pub.publish(l_hip_r_msg);

    r_hip_p_pub.publish(r_hip_p_msg);
    l_hip_p_pub.publish(l_hip_p_msg);

    r_kn_p_pub.publish(r_kn_p_msg);
    l_kn_p_pub.publish(l_kn_p_msg);

    r_an_p_pub.publish(r_an_p_msg);
    l_an_p_pub.publish(l_an_p_msg);

    r_an_r_pub.publish(r_an_r_msg);
    l_an_r_pub.publish(l_an_r_msg);

    ros::spinOnce();
    loop_rate.sleep();

  }

  ROS_INFO("Initial pose is reached!");

  rleg_cur_joint_pos_ = rleg_des_joint_pos_;
  lleg_cur_joint_pos_ = lleg_des_joint_pos_;
  //pelvis_cur_pose = pelvis_current_pose;

//Move COM
/*
double hip_roll;
double ankle_roll;

ROS_INFO("Start to move COM");

//pelvis_current_position = 0.0;
pelvis_current_position = pelvis_current_pose.p.y();

while(pelvis_current_position < lleg_current_pose.p.y()){

      pelvis_current_position += step;

      this->moveCOMToLeftLeg(KDL::Frame(pelvis_des_pose.M,
                                        KDL::Vector(lleg_current_pose.p.x(),
                                                     pelvis_current_position,
                                                     pelvis_target_position_z)),
                             lleg_des_joint_pos_);


      l_an_r_msg.data = -lleg_des_joint_pos_(0);
      ankle_roll = -lleg_des_joint_pos_(0);
      l_an_p_msg.data = lleg_des_joint_pos_(1);
      l_kn_p_msg.data = lleg_des_joint_pos_(2);
      l_hip_p_msg.data = lleg_des_joint_pos_(3);
      l_hip_r_msg.data = -lleg_des_joint_pos_(4);
      hip_roll = -lleg_des_joint_pos_(4);
      l_hip_y_msg.data = lleg_des_joint_pos_(5);

      l_hip_y_pub.publish(l_hip_y_msg);
      l_hip_r_pub.publish(l_hip_r_msg);
      l_hip_p_pub.publish(l_hip_p_msg);
      l_kn_p_pub.publish(l_kn_p_msg);
      l_an_p_pub.publish(l_an_p_msg);
      l_an_r_pub.publish(l_an_r_msg);

      ros::spinOnce();
      loop_rate.sleep();


}
*/


//hip roll adjustment
/*
ROS_INFO("Starting adjustment!");
while(true){
  hip_roll+=0.001;
  l_hip_r_msg.data = hip_roll;
  l_hip_r_pub.publish(l_hip_r_msg);
  ROS_INFO("Doing some fit");
  ros::spinOnce();
  loop_rate.sleep();


}
*/

//ROS_INFO("COM is translated");

  return true;

}
