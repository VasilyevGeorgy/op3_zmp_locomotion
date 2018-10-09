#include "op3_zmp_locomotion.h"

void op3_zmp_locomotion::op3_left_leg(KDL::Frame pelvis_pose, KDL::Frame lfoot_pose){
//foot->pelvis
  lleg_foot_to_pelvis_chain.addSegment(KDL::Segment("base", // Origin
                                       KDL::Joint(KDL::Joint::None),
                                       lfoot_pose, // from base to foot
                                       KDL::RigidBodyInertia(0.0,
                                                             KDL::Vector(0.0, 0.0, 0.0), // KDL::Vector(0.0, 0.0, 0.0) or KDL::Vector(0.0, 0.035, 0.0)
                                                             KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                             )
                                       )
                          );
//2
  lleg_foot_to_pelvis_chain.addSegment(KDL::Segment("l_leg_end", // !!! probably, it's lower surface of foot
                                       KDL::Joint(KDL::Joint::None),
                                       KDL::Frame(KDL::Vector(0.0 , 0.0 , 0.0305)), // from foot to ankle_roll joit
                                       KDL::RigidBodyInertia(0.0,
                                                             KDL::Vector(0.0, 0.0, 0.0),
                                                             KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                             )
                                       )
                          );

//3
  lleg_foot_to_pelvis_chain.addSegment(KDL::Segment("l_leg_an_r",
                                       KDL::Joint(KDL::Joint::RotX),
                                       KDL::Frame(KDL::Vector(0.0, 0.0, 0.0)),
                                       KDL::RigidBodyInertia(0.06934,
                                                             KDL::Vector(0.0, 0.0, 0.0),
                                                             KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                             )
                                       )
                          );
//4
  lleg_foot_to_pelvis_chain.addSegment(KDL::Segment("l_leg_an_p",
                                       KDL::Joint("minus_RotY", KDL::Vector(0,0,0), KDL::Vector(0,-1,0), KDL::Joint::RotAxis),  ///!!!
                                       KDL::Frame(KDL::Vector(0.0, 0.0, 0.11)),
                                       KDL::RigidBodyInertia(0.17886,
                                                             KDL::Vector(0.0, 0.0, 0.0),
                                                             KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                             )
                                       )
                          );
//5
  lleg_foot_to_pelvis_chain.addSegment(KDL::Segment("l_leg_kn_p",
                                       KDL::Joint(KDL::Joint::RotY), ///!!!
                                       KDL::Frame(KDL::Vector(0.0, 0.0, 0.11)),
                                       KDL::RigidBodyInertia(0.04015,
                                                             KDL::Vector(0.0, 0.0, 0.0),
                                                             KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                             )
                                       )
                          );
//6
  lleg_foot_to_pelvis_chain.addSegment(KDL::Segment("l_leg_hip_p",
                                       KDL::Joint(KDL::Joint::RotY), ///!!!
                                       KDL::Frame(KDL::Vector(0.0, 0.0, 0.0)),
                                       KDL::RigidBodyInertia(0.11543,
                                                             KDL::Vector(0.0, 0.0, 0.0),
                                                             KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                             )
                                       )
                          );
//7
  lleg_foot_to_pelvis_chain.addSegment(KDL::Segment("l_leg_hip_r",
                                       KDL::Joint("minus_RotX", KDL::Vector(0,0,0), KDL::Vector(-1,0,0), KDL::Joint::RotAxis),
                                       KDL::Frame(KDL::Vector(0.0, 0.0, 0.0285)),
                                       KDL::RigidBodyInertia(0.17886,
                                                             KDL::Vector(0.0, 0.0, 0.0),
                                                             KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                             )
                                       )
                          );
//8
  lleg_foot_to_pelvis_chain.addSegment(KDL::Segment("l_hip_yaw",
                                       KDL::Joint("minus_RotZ", KDL::Vector(0,0,0), KDL::Vector(0,0,-1), KDL::Joint::RotAxis),
                                       KDL::Frame(KDL::Vector(0.000, -0.035, 0.0907)),
                                       KDL::RigidBodyInertia(0.01181,
                                                             KDL::Vector(0.0, 0.0, 0.0),
                                                             KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                             )
                                       )
                          );
//9
  lleg_foot_to_pelvis_chain.addSegment(KDL::Segment("pelvis", // end of left leg
                                       KDL::Joint(KDL::Joint::None),
                                       KDL::Frame(KDL::Vector(0.0,0.0,0.0)),
                                       KDL::RigidBodyInertia(0.72235,
                                                             KDL::Vector(0.0, 0.0, 0.0),
                                                             KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                             )
                                      )
                          );
//From pelvis to foot
  lleg_pelvis_to_foot_chain.addSegment(KDL::Segment("base",
                                       KDL::Joint(KDL::Joint::None),
                                       pelvis_pose,
                                       KDL::RigidBodyInertia(0.0,
                                                             KDL::Vector(0.0, 0.0, 0.0),
                                                             KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                             )
                                       )
                          );
  lleg_pelvis_to_foot_chain.addSegment(KDL::Segment("pelvis",
                                       KDL::Joint(KDL::Joint::None),
                                       KDL::Frame(KDL::Vector(0.0, 0.035, -0.0907)),
                                       KDL::RigidBodyInertia(0.72235,
                                                             KDL::Vector(0.0, 0.0, 0.0),
                                                             KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                             )
                                       )
                          );
  lleg_pelvis_to_foot_chain.addSegment(KDL::Segment("l_leg_hip_y",
                                       KDL::Joint("minus_RotZ", KDL::Vector(0,0,0), KDL::Vector(0,0,-1), KDL::Joint::RotAxis),
                                       KDL::Frame(KDL::Vector(0.000, 0.000, -0.0285)),
                                       KDL::RigidBodyInertia(0.01181,
                                                             KDL::Vector(0.0, 0.0, 0.0),
                                                             KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                             )
                                       )
                          );
  lleg_pelvis_to_foot_chain.addSegment(KDL::Segment("l_leg_hip_r",
                                       KDL::Joint("minus_RotX", KDL::Vector(0,0,0), KDL::Vector(-1,0,0), KDL::Joint::RotAxis),
                                       KDL::Frame(KDL::Vector(0.0, 0.0, 0.0)),
                                       KDL::RigidBodyInertia(0.17886,
                                                             KDL::Vector(0.0, 0.0, 0.0),
                                                             KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                             )
                                       )
                          );
  lleg_pelvis_to_foot_chain.addSegment(KDL::Segment("l_leg_hip_p",
                                       KDL::Joint(KDL::Joint::RotY),
                                       KDL::Frame(KDL::Vector(0.0, 0.0, -0.11)),
                                       KDL::RigidBodyInertia(0.11543,
                                                             KDL::Vector(0.0, 0.0, 0.0),
                                                             KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                             )
                                       )
                          );
  lleg_pelvis_to_foot_chain.addSegment(KDL::Segment("l_leg_kn_p",
                                       KDL::Joint(KDL::Joint::RotY),
                                       KDL::Frame(KDL::Vector(0.0, 0.0, -0.11)),
                                       KDL::RigidBodyInertia(0.04015,
                                                             KDL::Vector(0.0, 0.0, 0.0),
                                                             KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                             )
                                       )
                          );
  lleg_pelvis_to_foot_chain.addSegment(KDL::Segment("l_leg_an_p",
                                       KDL::Joint("minus_RotY", KDL::Vector(0,0,0), KDL::Vector(0,-1,0), KDL::Joint::RotAxis),
                                       KDL::Frame(KDL::Vector(0.0, 0.0, 0.0)),
                                       KDL::RigidBodyInertia(0.17886,
                                                             KDL::Vector(0.0, 0.0, 0.0),
                                                             KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                             )
                                       )
                          );
  lleg_pelvis_to_foot_chain.addSegment(KDL::Segment("l_leg_an_r",
                                       KDL::Joint(KDL::Joint::RotX),
                                       KDL::Frame(KDL::Vector(0.0, 0.0, -0.0305)),
                                       KDL::RigidBodyInertia(0.06934,
                                                             KDL::Vector(0.0, 0.0, 0.0),
                                                             KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                             )
                                       )
                          );
  lleg_pelvis_to_foot_chain.addSegment(KDL::Segment("l_leg_end",
                                       KDL::Joint(KDL::Joint::None),
                                       KDL::Frame(KDL::Vector(0.0 , 0.0 , 0.0)),
                                       KDL::RigidBodyInertia(0.0,
                                                             KDL::Vector(0.0, 0.0, 0.0),
                                                             KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                             )
                                       )
                          );

}
