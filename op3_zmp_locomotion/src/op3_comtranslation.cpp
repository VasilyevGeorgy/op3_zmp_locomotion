#include "op3_zmp_locomotion.h"

void op3_zmp_locomotion::comTranslation(Eigen::VectorXd &rleg_cur_joint_pos_, Eigen::VectorXd &lleg_cur_joint_pos_){


  Eigen::VectorXd rleg_joint_pos_, lleg_joint_pos_;

  rleg_joint_pos_.resize(JOINT_NUM);
  lleg_joint_pos_.resize(JOINT_NUM);

//Initialize desired joint position
  Eigen::VectorXd rleg_des_joint_pos_;
  Eigen::VectorXd lleg_des_joint_pos_;

  rleg_des_joint_pos_.resize(JOINT_NUM);
  lleg_des_joint_pos_.resize(JOINT_NUM);

  for (int i=0;i<6;i++)
  {
    rleg_joint_pos_(i) = rleg_cur_joint_pos_(i);
    lleg_joint_pos_(i) = lleg_cur_joint_pos_(i);
//if NOT changed during CoM translating
    rleg_des_joint_pos_(i) = rleg_cur_joint_pos_(i);
    lleg_des_joint_pos_(i) = lleg_cur_joint_pos_(i);
  }

  this->setJointPosition(rleg_joint_pos_,lleg_joint_pos_);

//Move COM

  ROS_INFO("\nStart to move COM");

  ROS_INFO("\n\n\nLeft leg (deg) an_r:%f, an_p:%f, kn_p:%f, hip_p:%f, hip_r:%f, hip_yaw:%f\n\n\n",
           lleg_joint_position.data(0)*R2D,lleg_joint_position.data(1)*R2D,lleg_joint_position.data(2)*R2D,
           lleg_joint_position.data(3)*R2D,lleg_joint_position.data(4)*R2D,lleg_joint_position.data(5)*R2D
           );

  ROS_INFO("lleg_current_pose_x: %f; lleg_current_pose_y: %f", lleg_current_pose.p.x(), lleg_current_pose.p.y());

  double pelvis_current_position_x = 0.0;
  double pelvis_current_position_y = pelvis_current_pose.p.y(); // pelvis_current_pose.p.y();

  ros::Rate loop_rate(80);

  double step = 0.0002;

  if (this->init_leg == "right" || this->init_leg == "Right"){

      //ROS_INFO("%s", this->init_leg.c_str());

      while(pelvis_current_position_y < lleg_current_pose.p.y() && pelvis_current_position_x < lleg_current_pose.p.x()){

            pelvis_current_position_x += step;
            pelvis_current_position_y += step;

            ROS_INFO("Left leg (deg) an_r:%f, an_p:%f, kn_p:%f, hip_p:%f, hip_r:%f, hip_yaw:%f\n",
                     lleg_des_joint_pos_(0)*R2D,lleg_des_joint_pos_(1)*R2D,lleg_des_joint_pos_(2)*R2D,
                     lleg_des_joint_pos_(3)*R2D,lleg_des_joint_pos_(4)*R2D,lleg_des_joint_pos_(5)*R2D
                     );


            this->moveCOMToLeftLeg(KDL::Frame(pelvis_desired_pose.M,
                                              KDL::Vector(pelvis_current_position_x,
                                                           pelvis_current_position_y,
                                                           this->pelvis_position_z)),
                                   lleg_des_joint_pos_);

            ROS_INFO("Left leg (deg) an_r:%f, an_p:%f, kn_p:%f, hip_p:%f, hip_r:%f, hip_yaw:%f\n",
                     lleg_des_joint_pos_(0)*R2D,lleg_des_joint_pos_(1)*R2D,lleg_des_joint_pos_(2)*R2D,
                     lleg_des_joint_pos_(3)*R2D,lleg_des_joint_pos_(4)*R2D,lleg_des_joint_pos_(5)*R2D
                     );


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

      while(pelvis_current_position_y > rleg_current_pose.p.y()){

            pelvis_current_position_y -= step;

            this->moveCOMToRightLeg(KDL::Frame(pelvis_desired_pose.M,
                                              KDL::Vector(rleg_current_pose.p.x(),
                                                           pelvis_current_position_y,
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

  for (int i=0;i<6;i++)
  {
    rleg_cur_joint_pos_(i) = rleg_des_joint_pos_(i);
    lleg_cur_joint_pos_(i) = lleg_des_joint_pos_(i);
  }

  //ROS_INFO("test\n r_hip_y: %f;   l_hip_y: %f\n r_hip_r: %f;   l_hip_r: %f\n r_hip_p: %f;   l_hip_p: %f\n r_kn_p: %f;   l_kn_p: %f\n r_an_p: %f;   l_an_p: %f\n r_an_r: %f;   l_an_r: %f\n",
  //         rleg_cur_joint_pos_(5)*R2D,lleg_cur_joint_pos_(5)*R2D,rleg_cur_joint_pos_(4)*R2D,lleg_cur_joint_pos_(4)*R2D,rleg_cur_joint_pos_(3)*R2D,lleg_cur_joint_pos_(3)*R2D,
  //         rleg_cur_joint_pos_(2)*R2D,lleg_cur_joint_pos_(2)*R2D,rleg_cur_joint_pos_(1)*R2D,lleg_cur_joint_pos_(1)*R2D,rleg_cur_joint_pos_(0)*R2D,lleg_cur_joint_pos_(0)*R2D);

}
