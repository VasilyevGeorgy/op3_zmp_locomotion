#include "op3_zmp_locomotion.h"

void op3_zmp_locomotion::comTranslation(Eigen::VectorXd &rleg_cur_joint_pos_, Eigen::VectorXd &lleg_cur_joint_pos_){

  Eigen::VectorXd rleg_joint_pos_, lleg_joint_pos_;

  rleg_joint_pos_.resize(JOINT_NUM);
  rleg_joint_pos_(0) = rleg_cur_joint_pos_(0);
  rleg_joint_pos_(1) = rleg_cur_joint_pos_(1);
  rleg_joint_pos_(2) = rleg_cur_joint_pos_(2);
  rleg_joint_pos_(3) = rleg_cur_joint_pos_(3);
  rleg_joint_pos_(4) = rleg_cur_joint_pos_(4);
  rleg_joint_pos_(5) = rleg_cur_joint_pos_(5);

  lleg_joint_pos_.resize(JOINT_NUM);
  lleg_joint_pos_(0) = lleg_cur_joint_pos_(0);
  lleg_joint_pos_(1) = lleg_cur_joint_pos_(1);
  lleg_joint_pos_(2) = lleg_cur_joint_pos_(2);
  lleg_joint_pos_(3) = lleg_cur_joint_pos_(3);
  lleg_joint_pos_(4) = lleg_cur_joint_pos_(4);
  lleg_joint_pos_(5) = lleg_cur_joint_pos_(5);

  this->setJointPosition(rleg_joint_pos_,lleg_joint_pos_);

  Eigen::VectorXd rleg_des_joint_pos_;
  Eigen::VectorXd lleg_des_joint_pos_;

//Move COM

  double hip_roll;
  double ankle_roll;

  ROS_INFO("Start to move COM");

  double pelvis_current_position = pelvis_current_pose.p.y();

  ros::Rate loop_rate(20);

  double step = 0.0002;

  if (this->init_leg == "right" || this->init_leg == "Right"){

      while(pelvis_current_position < lleg_current_pose.p.y()){

            pelvis_current_position += step;

            this->moveCOMToLeftLeg(KDL::Frame(pelvis_desired_pose.M,
                                              KDL::Vector(lleg_current_pose.p.x(),
                                                           pelvis_current_position,
                                                           this->pelvis_position_z)),
                                   lleg_des_joint_pos_);


            l_an_r_msg.data =  lleg_des_joint_pos_(0);
            //ankle_roll = -lleg_des_joint_pos_(0);
            l_an_p_msg.data =  lleg_des_joint_pos_(1);
            l_kn_p_msg.data =  lleg_des_joint_pos_(2);
            l_hip_p_msg.data = lleg_des_joint_pos_(3);
            l_hip_r_msg.data = lleg_des_joint_pos_(4);
            //hip_roll = -lleg_des_joint_pos_(4);
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
  }
  else{
    if (this->init_leg == "left" || this->init_leg == "Left"){

      while(pelvis_current_position > rleg_current_pose.p.y()){

            pelvis_current_position -= step;

            this->moveCOMToRightLeg(KDL::Frame(pelvis_desired_pose.M,
                                              KDL::Vector(rleg_current_pose.p.x(),
                                                           pelvis_current_position,
                                                           this->pelvis_position_z)),
                                   rleg_des_joint_pos_);


            r_an_r_msg.data =  -rleg_des_joint_pos_(0);
            //ankle_roll = -rleg_des_joint_pos_(0);
            r_an_p_msg.data =  -rleg_des_joint_pos_(1);
            r_kn_p_msg.data =  -rleg_des_joint_pos_(2);
            r_hip_p_msg.data = -rleg_des_joint_pos_(3);
            r_hip_r_msg.data = -rleg_des_joint_pos_(4);
            //hip_roll = -rleg_des_joint_pos_(4);
            r_hip_y_msg.data =  rleg_des_joint_pos_(5);

            r_hip_y_pub.publish(r_hip_y_msg);
            r_hip_r_pub.publish(r_hip_r_msg);
            r_hip_p_pub.publish(r_hip_p_msg);
            r_kn_p_pub.publish(r_kn_p_msg);
            r_an_p_pub.publish(r_an_p_msg);
            r_an_r_pub.publish(r_an_r_msg);

            ros::spinOnce();
            loop_rate.sleep();

      }
    }
    else
      ROS_INFO("Please choose initial swing leg properly!");
  }

}
