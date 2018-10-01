#include"3dof_planar_man.h"

Planar_man::Planar_man(){

  init_joint_position.resize(JOINT_NUM);
  init_joint_position.data = Eigen::Vector3d(M_PI/4, -M_PI/2, 0.0);

// Set Kinematic Chain
  m_chain.addSegment(KDL::Segment("base", // Origin
                                     KDL::Joint(KDL::Joint::None),
                                     KDL::Frame(KDL::Vector(0.0 , 0.0, 0.0)), // from base to first joint
                                     KDL::RigidBodyInertia(0.0,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
//1
  m_chain.addSegment(KDL::Segment("first_link",
                                     KDL::Joint(KDL::Joint::RotX),
                                     KDL::Frame(KDL::Vector(0.0 , 0.0 , 1.0)), // KDL::Frame(KDL::Rotation::EulerZYX(0.0, 0.0, M_PI/4)) * KDL::Frame(KDL::Vector(0.0 , 0.0 , 1.0))
                                     KDL::RigidBodyInertia(0.0, // !!!
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );

//2
  m_chain.addSegment(KDL::Segment("sec_link",
                                     KDL::Joint(KDL::Joint::RotX),
                                     KDL::Frame(KDL::Vector(0.0, 0.0, 1.0)),
                                     KDL::RigidBodyInertia(0.0, // !!!
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
//3
  m_chain.addSegment(KDL::Segment("tip",
                                     KDL::Joint(KDL::Joint::RotX), // RotX
                                     KDL::Frame(KDL::Vector(0.0, 0.0, 0.0)),
                                     KDL::RigidBodyInertia(0.0,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );

  min_position_limit.push_back(-180.0); max_position_limit.push_back(180.0);
  min_position_limit.push_back(-180.0); max_position_limit.push_back(180.0);
  min_position_limit.push_back(-180.0); max_position_limit.push_back(180.0);

//Convert angles to rads
      min_joint_position_limit.resize(JOINT_NUM);
      max_joint_position_limit.resize(JOINT_NUM);
      for (int index=0; index<JOINT_NUM; index++)
      {
        min_joint_position_limit(index) = min_position_limit[index]*D2R; //D2R - degrees to radians
        max_joint_position_limit(index) = max_position_limit[index]*D2R;
      }

      m_fk_solver_ = new KDL::ChainFkSolverPos_recursive(m_chain);

      m_ik_vel_solver_ = new KDL::ChainIkSolverVel_pinv(m_chain);
      m_ik_pos_solver_ = new KDL::ChainIkSolverPos_NR_JL(m_chain,
                                                          min_joint_position_limit,max_joint_position_limit,
                                                          *m_fk_solver_,
                                                          *m_ik_vel_solver_);

      is_initialized = false;
}

Planar_man::~Planar_man()
{
  delete m_fk_solver_;
  delete m_ik_vel_solver_;
  delete m_ik_pos_solver_;
}

void Planar_man::setInitialPosition(double_t th1,
                                      double_t th2,
                                      double_t th3){

  if (fabs(th1*D2R)+fabs(th2*D2R)+fabs(th3*D2R)>0.001){ // initial position's singularity check
    is_initialized = true;
    init_joint_position.data = Eigen::Vector3d(th1*D2R, th2*D2R, th3*D2R);
  }

}

std::vector<double_t> Planar_man::solveIK(Eigen::Vector3d coordinates,Eigen::Vector3d  rpy){

  tip_desired_pose = KDL::Frame(KDL::Rotation::RPY(rpy[0],rpy[1],rpy[2]),
                                  KDL::Vector(coordinates[0],
                                                coordinates[1],
                                                coordinates[2]));
  desired_joint_position.resize(JOINT_NUM);

  if(!is_initialized) init_joint_position.data = Eigen::Vector3d(M_PI/4, -M_PI/2, -M_PI/4); // default initial position

  int ik_pose_err = m_ik_pos_solver_->CartToJnt(init_joint_position, tip_desired_pose, desired_joint_position);

  if (ik_pose_err != 0)
  {
    ROS_WARN("IK ERR : %s", m_ik_pos_solver_->strError(ik_pose_err));
  }

// output
  if(ik_pose_err>=0){

    IK_pos_output.resize(JOINT_NUM);

    for (int i=0; i<JOINT_NUM; i++)
    {
      IK_pos_output[i] = desired_joint_position(i);

    }

    ROS_INFO("Inverse Kinematics:");
    ROS_INFO("Joint angles: %f, %f, %f",
             IK_pos_output[0]*R2D,IK_pos_output[1]*R2D,IK_pos_output[2]*R2D);
  }

  return IK_pos_output;
}

std::vector<double_t> Planar_man::solveFK(Eigen::Vector3d jnt_pose){

  KDL::JntArray jnt_fk = KDL::JntArray(JOINT_NUM);

  for(int i=0;i<JOINT_NUM;i++){
    jnt_fk(i) = jnt_pose[i]*D2R;
  }

  m_fk_solver_->JntToCart(jnt_fk, m_tip_pose);

  double roll, pitch, yaw;

  m_tip_pose.M.GetRPY(roll,pitch,yaw);
  ROS_INFO(" ");
  ROS_INFO("Forward Kinematics:");
  ROS_INFO("X: %f, Y: %f, Z: %f",
           m_tip_pose.p.x(),m_tip_pose.p.y(),m_tip_pose.p.z());
  ROS_INFO("Roll: %f, Pitch: %f, Yaw: %f",
          roll,pitch,yaw);
  ROS_INFO(" ");

  FK_output.resize(JOINT_NUM);
  FK_output[0] = m_tip_pose.p.y();
  FK_output[1] = m_tip_pose.p.z();
  FK_output[2] = roll;

  return FK_output;

}

std::vector<double_t> Planar_man::solveFK(){

  KDL::JntArray jnt_fk = KDL::JntArray(JOINT_NUM);

  if(!is_initialized){

    init_joint_position.data = Eigen::Vector3d(M_PI/4, -M_PI/2, -M_PI/4); // default initial position
    ROS_INFO("Default joint position is used");
  }
  jnt_fk = init_joint_position;

  m_fk_solver_->JntToCart(jnt_fk, m_tip_pose);

  double roll, pitch, yaw;

  m_tip_pose.M.GetRPY(roll,pitch,yaw);
  ROS_INFO("Forward Kinematics:");
  ROS_INFO("X: %f, Y: %f, Z: %f",
           m_tip_pose.p.x(),m_tip_pose.p.y(),m_tip_pose.p.z());
  ROS_INFO("Roll:%.3f, Pitch:%.3f, Yaw:%.3f",
          roll*R2D,pitch*R2D,yaw*R2D);

  FK_output.resize(JOINT_NUM);
  FK_output[0] = m_tip_pose.p.y();
  FK_output[1] = m_tip_pose.p.z();
  FK_output[2] = roll;

  return FK_output;


}

