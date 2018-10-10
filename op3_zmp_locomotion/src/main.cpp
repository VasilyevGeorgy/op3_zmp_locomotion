#include "op3_zmp_locomotion.h"

int main (int argc, char **argv){

    ros::init(argc,argv,"set_initial_pose");

    op3_zmp_locomotion move_robot;

    move_robot.step_length = 50; //mm
    move_robot.clearance = 20; //mm
    move_robot.step_duration = 4.0; //s
    move_robot.pelvis_position_z = 0.34;
    move_robot.init_leg = "left";

    Eigen::VectorXd rleg_cur_joint_pos;
    Eigen::VectorXd lleg_cur_joint_pos;

    //Eigen::VectorXd &rleg_cur_joint_pos_ = rleg_cur_joint_pos;
    //Eigen::VectorXd &lleg_cur_joint_pos_ = lleg_cur_joint_pos;

    rleg_cur_joint_pos.resize(JOINT_NUM);
    lleg_cur_joint_pos.resize(JOINT_NUM);

    ros::Duration(1).sleep();

    move_robot.setInitPose(KDL::Frame(KDL::Rotation::RPY(0.0, 0.0, 0.0),
                                       KDL::Vector(0.0,0.0,move_robot.pelvis_position_z)), // z = 0.34
                                       rleg_cur_joint_pos,lleg_cur_joint_pos);


    move_robot.comTranslation(rleg_cur_joint_pos, lleg_cur_joint_pos);

    move_robot.legSwing(rleg_cur_joint_pos, lleg_cur_joint_pos);
    //
    //move_robot.init_leg = "right";
    //
    //move_robot.comTranslation(rleg_cur_joint_pos, lleg_cur_joint_pos);
    //
    //for (int i=0;i<JOINT_NUM-1;i++)
    //{
    //  move_robot.lleg_joint_position.data[i] = -lleg_cur_joint_pos(i);
    //  ROS_INFO("%f",move_robot.lleg_joint_position.data[i]*R2D);
    //}
    //move_robot.lleg_joint_position.data[JOINT_NUM-1] = lleg_cur_joint_pos(JOINT_NUM-1);
    //ROS_INFO("%f",move_robot.lleg_joint_position.data[JOINT_NUM-1]*R2D);
    //
    ////move_robot.moveCOMToLeftLeg(KDL::Frame(KDL::Rotation::RPY(0.0, 0.0, 0.0),
    ////                                       KDL::Vector(-0.05, // move_robot.lleg_current_pose.p.x()
    ////                                                   0.0, // move_robot.lleg_current_pose.p.y()
    ////                                                   move_robot.pelvis_position_z)),
    ////                            lleg_cur_joint_pos);



    for (int i=0;i<JOINT_NUM;i++)
    {
      move_robot.lleg_joint_position.data[i] = lleg_cur_joint_pos(i);
      ROS_INFO("%f",move_robot.lleg_joint_position.data[i]*R2D);
    }

    bool kinematics_status;
    KDL::Frame pos_test;
    kinematics_status = move_robot.lleg_foot_to_pelvis_fk_solver->JntToCart(move_robot.lleg_joint_position,pos_test); //lleg_foot_to_pelvis_fk_solver

    if(kinematics_status>=0){
        ROS_INFO("FK\nx: %f; y: %f, z: %f", pos_test.p.x(),pos_test.p.y(),pos_test.p.z());
    }else{
        ROS_WARN("FK error!");
    }

    move_robot.init_leg = "right";

    move_robot.comTranslation(rleg_cur_joint_pos, lleg_cur_joint_pos);


    return 0;

}
