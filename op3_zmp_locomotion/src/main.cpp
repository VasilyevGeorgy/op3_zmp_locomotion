#include "op3_zmp_locomotion.h"

int main (int argc, char **argv){

    ros::init(argc,argv,"set_initial_pose");

    op3_zmp_locomotion move_robot;

    move_robot.step_length = 50; //mm
    move_robot.clearance = 20; //mm
    move_robot.step_duration = 3.0; //s
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

    move_robot.init_leg = "right";

    //ROS_INFO("\n\n\nLeft leg (deg) an_r:%f, an_p:%f, kn_p:%f, hip_p:%f, hip_r:%f, hip_yaw:%f\n\n\n",
    //         lleg_cur_joint_pos[0]*R2D,lleg_cur_joint_pos[1]*R2D,lleg_cur_joint_pos[2]*R2D,
    //         lleg_cur_joint_pos[3]*R2D,lleg_cur_joint_pos[4]*R2D,lleg_cur_joint_pos[5]*R2D
    //         );

    move_robot.comTranslation(rleg_cur_joint_pos, lleg_cur_joint_pos);

    //move_robot.moveCOMToLeftLeg(KDL::Frame(KDL::Rotation::RPY(0.0, 0.0, 0.0),
    //                                       KDL::Vector(move_robot.lleg_current_pose.p.x(),
    //                                                   move_robot.lleg_current_pose.p.y(),
    //                                                   move_robot.pelvis_position_z)),
    //                            lleg_cur_joint_pos);
    //


    return 0;

}
