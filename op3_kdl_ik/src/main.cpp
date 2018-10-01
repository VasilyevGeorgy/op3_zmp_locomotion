#include "op3_kdl_move_pelvis.h"
#include "std_msgs/Float64.h"

#define PELVIS_INIT_POSITION (0.3697)

int main (int argc, char **argv){

  ros::init(argc,argv,"op3_move_pelvis");
  ros::NodeHandle node;

//Right leg
  ros::Publisher r_hip_y_pub = node.advertise<std_msgs::Float64>("robotis_op3/r_hip_yaw_position/command",10);
  ros::Publisher r_hip_r_pub = node.advertise<std_msgs::Float64>("robotis_op3/r_hip_roll_position/command",10);
  ros::Publisher r_hip_p_pub = node.advertise<std_msgs::Float64>("robotis_op3/r_hip_pitch_position/command",10);
  ros::Publisher r_kn_p_pub = node.advertise<std_msgs::Float64>("robotis_op3/r_knee_position/command",10);
  ros::Publisher r_an_p_pub = node.advertise<std_msgs::Float64>("robotis_op3/r_ank_pitch_position/command",10);
  ros::Publisher r_an_r_pub = node.advertise<std_msgs::Float64>("robotis_op3/r_ank_roll_position/command",10);
//Left leg
  ros::Publisher l_hip_y_pub = node.advertise<std_msgs::Float64>("robotis_op3/l_hip_yaw_position/command",10);
  ros::Publisher l_hip_r_pub = node.advertise<std_msgs::Float64>("robotis_op3/l_hip_roll_position/command",10);
  ros::Publisher l_hip_p_pub = node.advertise<std_msgs::Float64>("robotis_op3/l_hip_pitch_position/command",10);
  ros::Publisher l_kn_p_pub = node.advertise<std_msgs::Float64>("robotis_op3/l_knee_position/command",10);
  ros::Publisher l_an_p_pub = node.advertise<std_msgs::Float64>("robotis_op3/l_ank_pitch_position/command",10);
  ros::Publisher l_an_r_pub = node.advertise<std_msgs::Float64>("robotis_op3/l_ank_roll_position/command",10);


  ros::Rate loop_rate(10);

  op3_kdl_move_pelvis move_pelvis;

  Eigen::VectorXd rleg_joint_pos_, lleg_joint_pos_;

  rleg_joint_pos_.resize(JOINT_NUM);
  rleg_joint_pos_(0) = 0.001;
  rleg_joint_pos_(1) = -0.002;
  rleg_joint_pos_(2) = 0.001;
  rleg_joint_pos_(3) = 0.00;
  rleg_joint_pos_(4) = 0.00;
  rleg_joint_pos_(5) = 0.00;

  lleg_joint_pos_.resize(JOINT_NUM);
  lleg_joint_pos_(0) = 0.001;
  lleg_joint_pos_(1) = -0.02;
  lleg_joint_pos_(2) = 0.001;
  lleg_joint_pos_(3) = 0.0;
  lleg_joint_pos_(4) = 0.0;
  lleg_joint_pos_(5) = 0.0;

  move_pelvis.setJointPosition(rleg_joint_pos_,lleg_joint_pos_);

  Eigen::VectorXd rleg_des_joint_pos_;
  Eigen::VectorXd lleg_des_joint_pos_;

  double pelvis_curren_position = 0.3697;
  double pelvis_target_position = 0.25;
  double step = 0.0002;

  while (pelvis_curren_position > pelvis_target_position){
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

    move_pelvis.solveInverseKinematics(KDL::Frame(KDL::Rotation::RPY(0.0, 0.0, 0.0),
                                       KDL::Vector(0.0, 0.1, pelvis_curren_position)), // zmax = 0.3697, zmin = 0.16 if R=P=Y=0
                                           rleg_des_joint_pos_, lleg_des_joint_pos_);
//Right leg
    r_an_r_msg.data = rleg_des_joint_pos_(0);
    r_an_p_msg.data = rleg_des_joint_pos_(1);
    r_kn_p_msg.data = rleg_des_joint_pos_(2);
    r_hip_p_msg.data = rleg_des_joint_pos_(3);
    r_hip_r_msg.data = rleg_des_joint_pos_(4);
    r_hip_y_msg.data = rleg_des_joint_pos_(5);
//Left leg
    l_an_r_msg.data = lleg_des_joint_pos_(0);
    l_an_p_msg.data = -lleg_des_joint_pos_(1);
    l_kn_p_msg.data = -lleg_des_joint_pos_(2);
    l_hip_p_msg.data = -lleg_des_joint_pos_(3);
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

    pelvis_curren_position -= step;

  }

  ROS_INFO("Action is done!");


  //move_pelvis.solveInverseKinematics(KDL::Frame(KDL::Rotation::RPY(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.0, 0.16)), // zmax = 0.3697, zmin = 0.16 if R=P=Y=0
  //                                       rleg_des_joint_pos_, lleg_des_joint_pos_);

  return 0;
}
