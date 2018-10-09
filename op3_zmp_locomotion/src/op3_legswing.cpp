#include "op3_zmp_locomotion.h"

void op3_zmp_locomotion::legSwing(Eigen::VectorXd &rleg_cur_joint_pos_, Eigen::VectorXd &lleg_cur_joint_pos_){

  ROS_INFO("\n\n\nLeg swing initialized! \n\n\n");
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
    rleg_joint_pos_(i) = rleg_cur_joint_pos_((JOINT_NUM-1)-i);
    lleg_joint_pos_(i) = lleg_cur_joint_pos_((JOINT_NUM-1)-i);
//if NOT changed during CoM translating
    rleg_des_joint_pos_(i) = rleg_cur_joint_pos_((JOINT_NUM-1)-i);
    lleg_des_joint_pos_(i) = lleg_cur_joint_pos_((JOINT_NUM-1)-i);
  }

  //ROS_INFO("\n\n\nLeft leg (deg):%f, %f, %f, %f, %f, %f\n\n\n",
  //         lleg_joint_position.data(0)*R2D,lleg_joint_position.data(1)*R2D,lleg_joint_position.data(2)*R2D,
  //         lleg_joint_position.data(3)*R2D,lleg_joint_position.data(4)*R2D,lleg_joint_position.data(5)*R2D
  //         );

  this->setJointPosition(rleg_joint_pos_,lleg_joint_pos_);

  //ROS_INFO("\n\n\nLeft leg (deg):%f, %f, %f, %f, %f, %f\n\n\n",
  //         lleg_joint_position.data(0)*R2D,lleg_joint_position.data(1)*R2D,lleg_joint_position.data(2)*R2D,
  //         lleg_joint_position.data(3)*R2D,lleg_joint_position.data(4)*R2D,lleg_joint_position.data(5)*R2D
  //         );

  ROS_INFO("Start leg swing!");

  //double pelvis_current_position = pelvis_current_pose.p.y();

  double y_offset = 0.0;

  if (this->init_leg == "right" || this->init_leg == "Right")
    y_offset = -0.035;
  else{
    if (this->init_leg == "left" || this->init_leg == "Left")
      y_offset = 0.035;
    else
      return;
  }

  double freq = 20;
  ros::Rate loop_rate(100);

  double foot_up_time = this->step_duration/4;
  ROS_INFO("foot_up_time: %f", foot_up_time);
  double foot_transl_time = this->step_duration/2;
  double foot_down_time = this->step_duration/4;

  double cur_time = 0.0; //ms
  double t_param = 0.0;

  double x_t = 0.0;
  double y_t = 0.0;
  double z_t = 0.0;

  int counter = 0;

  while (cur_time < this->step_duration){ // cur_time<=foot_up_time

    cur_time = counter/freq/5;
    ROS_INFO("cur_time: %f,   cur_step: %d", cur_time, counter);


    if (cur_time<foot_up_time){

      t_param = M_PI/2*cur_time/foot_up_time;

      ROS_INFO("t_param: %f", t_param);

      x_t = this->step_length/4*(1-cos(t_param))/1000;
      y_t = y_offset*sin(t_param) + y_offset;
      z_t = this->clearance*sin(t_param)/1000;

      //ROS_INFO("x_t: %f;   z_t: %f", x_t,z_t);

      if(y_offset == -0.035){

        this->moveRightLeg(KDL::Frame(KDL::Rotation::RPY(0.0, 0.0, 0.0),
                                          KDL::Vector(x_t, 0.0, z_t)), //Check it!!!
                               rleg_des_joint_pos_);

        r_an_r_msg.data =  rleg_des_joint_pos_(5);
        r_an_p_msg.data =  rleg_des_joint_pos_(4);
        r_kn_p_msg.data =  rleg_des_joint_pos_(3);
        r_hip_p_msg.data = rleg_des_joint_pos_(2);
        r_hip_r_msg.data = rleg_des_joint_pos_(1);
        r_hip_y_msg.data = rleg_des_joint_pos_(0);

        r_hip_y_pub.publish(r_hip_y_msg);
        r_hip_r_pub.publish(r_hip_r_msg);
        r_hip_p_pub.publish(r_hip_p_msg);
        r_kn_p_pub.publish(r_kn_p_msg);
        r_an_p_pub.publish(r_an_p_msg);
        r_an_r_pub.publish(r_an_r_msg);

        rleg_current_pose.p.data[0] = x_t;
        rleg_current_pose.p.data[1] = -y_t;
        rleg_current_pose.p.data[2] = z_t;

      }
      else{
        this->moveLeftLeg(KDL::Frame(KDL::Rotation::RPY(0.0, 0.0, 0.0),
                                          KDL::Vector(x_t, y_t, z_t)), // -y_t
                               lleg_des_joint_pos_);

        l_an_r_msg.data =  lleg_des_joint_pos_(5);
        l_an_p_msg.data =  lleg_des_joint_pos_(4);
        l_kn_p_msg.data =  lleg_des_joint_pos_(3);
        l_hip_p_msg.data = lleg_des_joint_pos_(2);
        l_hip_r_msg.data = lleg_des_joint_pos_(1);
        l_hip_y_msg.data = lleg_des_joint_pos_(0);

        l_hip_y_pub.publish(l_hip_y_msg);
        l_hip_r_pub.publish(l_hip_r_msg);
        l_hip_p_pub.publish(l_hip_p_msg);
        l_kn_p_pub.publish(l_kn_p_msg);
        l_an_p_pub.publish(l_an_p_msg);
        l_an_r_pub.publish(l_an_r_msg);

        lleg_current_pose.p.data[0] = x_t;
        lleg_current_pose.p.data[1] = y_t;
        lleg_current_pose.p.data[2] = z_t;

      }

    }

    if (foot_up_time<cur_time && cur_time<= (foot_up_time + foot_transl_time)){

      x_t = cur_time/this->step_duration*this->step_length/1000;
      y_t = 2*y_offset;
      z_t = this->clearance/1000;

      if(y_offset == -0.035){

        this->moveRightLeg(KDL::Frame(KDL::Rotation::RPY(0.0, 0.0, 0.0),
                                          KDL::Vector(x_t, y_t, z_t)), //Check it!!!
                               rleg_des_joint_pos_);

        r_an_r_msg.data =  rleg_des_joint_pos_(5);
        r_an_p_msg.data =  rleg_des_joint_pos_(4);
        r_kn_p_msg.data =  rleg_des_joint_pos_(3);
        r_hip_p_msg.data = rleg_des_joint_pos_(2);
        r_hip_r_msg.data = rleg_des_joint_pos_(1);
        r_hip_y_msg.data = rleg_des_joint_pos_(0);

        r_hip_y_pub.publish(r_hip_y_msg);
        r_hip_r_pub.publish(r_hip_r_msg);
        r_hip_p_pub.publish(r_hip_p_msg);
        r_kn_p_pub.publish(r_kn_p_msg);
        r_an_p_pub.publish(r_an_p_msg);
        r_an_r_pub.publish(r_an_r_msg);

        rleg_current_pose.p.data[0] = x_t;
        rleg_current_pose.p.data[1] = -y_t;
        rleg_current_pose.p.data[2] = z_t;

      }
      else{
        this->moveLeftLeg(KDL::Frame(KDL::Rotation::RPY(0.0, 0.0, 0.0),
                                          KDL::Vector(x_t, y_t, z_t)),
                               lleg_des_joint_pos_);

        l_an_r_msg.data =  lleg_des_joint_pos_(5);
        l_an_p_msg.data =  lleg_des_joint_pos_(4);
        l_kn_p_msg.data =  lleg_des_joint_pos_(3);
        l_hip_p_msg.data = lleg_des_joint_pos_(2);
        l_hip_r_msg.data = lleg_des_joint_pos_(1);
        l_hip_y_msg.data = lleg_des_joint_pos_(0);

        l_hip_y_pub.publish(l_hip_y_msg);
        l_hip_r_pub.publish(l_hip_r_msg);
        l_hip_p_pub.publish(l_hip_p_msg);
        l_kn_p_pub.publish(l_kn_p_msg);
        l_an_p_pub.publish(l_an_p_msg);
        l_an_r_pub.publish(l_an_r_msg);

        lleg_current_pose.p.data[0] = x_t;
        lleg_current_pose.p.data[1] = y_t;
        lleg_current_pose.p.data[2] = z_t;

      }

      //ROS_INFO("cur_x: %f", -cur_time/this->step_duration*this->step_length/1000);
    }

    //x_t = 0.0;
    //z_t = 0.0;

    if ((foot_up_time + foot_transl_time) < cur_time && cur_time <= this->step_duration){

      t_param = M_PI/2*(cur_time-foot_up_time-foot_transl_time)/foot_down_time;

      ROS_INFO("t_param: %f!", t_param);

      x_t = this->step_length/4/1000*(4 - cos(t_param)); //   *(1-cos(t_param))/1000 + (this->step_length)/1000.*3/4;
      y_t = 2*y_offset ; //+ y_offset*sin(t_param)
      z_t = this->clearance*(1-sin(t_param))/1000;

      //ROS_INFO("x_t: %f; z_t: %f", x_t, z_t);

      if(y_offset == -0.035){

        this->moveRightLeg(KDL::Frame(KDL::Rotation::RPY(0.0, 0.0, 0.0),
                                          KDL::Vector(x_t, y_t, z_t)), //Check it!!!
                               rleg_des_joint_pos_);

        r_an_r_msg.data =  rleg_des_joint_pos_(5);
        r_an_p_msg.data =  rleg_des_joint_pos_(4);
        r_kn_p_msg.data =  rleg_des_joint_pos_(3);
        r_hip_p_msg.data = rleg_des_joint_pos_(2);
        r_hip_r_msg.data = rleg_des_joint_pos_(1);
        r_hip_y_msg.data = rleg_des_joint_pos_(0);

        r_hip_y_pub.publish(r_hip_y_msg);
        r_hip_r_pub.publish(r_hip_r_msg);
        r_hip_p_pub.publish(r_hip_p_msg);
        r_kn_p_pub.publish(r_kn_p_msg);
        r_an_p_pub.publish(r_an_p_msg);
        r_an_r_pub.publish(r_an_r_msg);

        rleg_current_pose.p.data[0] = x_t;
        rleg_current_pose.p.data[1] = -y_t;
        rleg_current_pose.p.data[2] = z_t;

      }
      else{
        this->moveLeftLeg(KDL::Frame(KDL::Rotation::RPY(0.0, 0.0, 0.0),
                                          KDL::Vector(x_t, y_t, z_t)),
                               lleg_des_joint_pos_);

        l_an_r_msg.data =  lleg_des_joint_pos_(5);
        l_an_p_msg.data =  lleg_des_joint_pos_(4);
        l_kn_p_msg.data =  lleg_des_joint_pos_(3);
        l_hip_p_msg.data = lleg_des_joint_pos_(2);
        l_hip_r_msg.data = lleg_des_joint_pos_(1);
        l_hip_y_msg.data = lleg_des_joint_pos_(0);

        l_hip_y_pub.publish(l_hip_y_msg);
        l_hip_r_pub.publish(l_hip_r_msg);
        l_hip_p_pub.publish(l_hip_p_msg);
        l_kn_p_pub.publish(l_kn_p_msg);
        l_an_p_pub.publish(l_an_p_msg);
        l_an_r_pub.publish(l_an_r_msg);

        lleg_current_pose.p.data[0] = x_t;
        lleg_current_pose.p.data[1] = y_t;
        lleg_current_pose.p.data[2] = z_t;

      }

    }

    ROS_INFO("x_t: %f;   y_t: %f", lleg_current_pose.p.x(), lleg_current_pose.p.y());

    counter++;

    ros::spinOnce();
    loop_rate.sleep();
  }

  for (int i=0;i<6;i++)
  {
    //rleg_cur_joint_pos_(i) = rleg_des_joint_pos_((JOINT_NUM-1)-i);
    //lleg_cur_joint_pos_(i) = lleg_des_joint_pos_((JOINT_NUM-1)-i);

    rleg_cur_joint_pos_(i) = rleg_des_joint_pos_((JOINT_NUM-1)-i);
    lleg_cur_joint_pos_(i) = lleg_des_joint_pos_((JOINT_NUM-1)-i);

    lleg_joint_position.data[i] = lleg_cur_joint_pos_(i);

  }

}
