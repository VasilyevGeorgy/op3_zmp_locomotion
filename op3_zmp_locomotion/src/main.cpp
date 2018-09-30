#include "op3_zmp_locomotion.h"

int main (int argc, char **argv){

    ros::init(argc,argv,"set_initial_pose");

    op3_zmp_locomotion move_robot;

    move_robot.step_length = 50; //mm
    move_robot.clearance = 20; //mm
    move_robot.step_duration = 2000; //ms
    move_robot.pelvis_position_z = 0.34;

    Eigen::VectorXd rleg_cur_joint_pos;
    Eigen::VectorXd lleg_cur_joint_pos;

    //Eigen::VectorXd &rleg_cur_joint_pos_ = rleg_cur_joint_pos;
    //Eigen::VectorXd &lleg_cur_joint_pos_ = lleg_cur_joint_pos;

    rleg_cur_joint_pos.resize(JOINT_NUM);
    lleg_cur_joint_pos.resize(JOINT_NUM);

    move_robot.setInitPose(KDL::Frame(KDL::Rotation::RPY(0.0, 0.0, 0.0),
                                       KDL::Vector(0.0,0.0,move_robot.pelvis_position_z)), // z = 0.34
                                       rleg_cur_joint_pos,lleg_cur_joint_pos);

    ROS_INFO("test: %f,  %f", rleg_cur_joint_pos[0], lleg_cur_joint_pos[0]);

    move_robot.comTranslation(rleg_cur_joint_pos, lleg_cur_joint_pos);

    return 0;

}
