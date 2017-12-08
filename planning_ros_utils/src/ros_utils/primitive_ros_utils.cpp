/**
 * @file primitive_ros_utils.cpp
 * @brief Cpp for interface between primitive classes and ros
 */
#include <ros_utils/primitive_ros_utils.h>

/**
 * @brief Primitive class to primitive ros message
 */
planning_ros_msgs::Primitive toPrimitiveROSMsg(const Primitive& pr) {
  planning_ros_msgs::Primitive msg;
  const Vec6f cx = pr.traj(0).coeff();
  const Vec6f cy = pr.traj(1).coeff();
  const Vec6f cz = pr.traj(2).coeff();
  msg.cx.resize(6);
  msg.cy.resize(6);
  msg.cz.resize(6);
  for(int i = 0; i < 6; i++) {
    msg.cx[i] = cx(i);
    msg.cy[i] = cy(i);
    msg.cz[i] = cz(i);
  }
  msg.t = pr.t();

  return msg;
}


/**
 * @brief Trajectory class to trajectory ros message
 */
planning_ros_msgs::Trajectory toTrajectoryROSMsg(const Trajectory& traj) {
  planning_ros_msgs::Trajectory msg;
  for(const auto& seg: traj.segs)
    msg.primitives.push_back(toPrimitiveROSMsg(seg));

  if(traj.lambda().exist()) {
    auto l = traj.lambda();
    msg.lambda.resize(l.segs.size());
    for(int i = 0; i < (int)l.segs.size(); i++) {
      msg.lambda[i].dT = l.segs[i].dT;
      msg.lambda[i].ti = l.segs[i].ti;
      msg.lambda[i].tf = l.segs[i].tf;
      msg.lambda[i].ca.resize(4);
      for(int j = 0; j < 4; j++)
        msg.lambda[i].ca[j] = l.segs[i].a(j);
    }
    //msg.t = l.getTotalTime();
  }
  return msg;
}

/**
 * @brief Multiple trajectories class to trajectories ros message for visualization
 */
planning_ros_msgs::Primitives toPrimitivesROSMsg(const std::vector<Primitive>& prs) {
  planning_ros_msgs::Primitives msg;
  for(const auto& pr: prs)
    msg.primitives.push_back(toPrimitiveROSMsg(pr));
  return msg;
}

/**
 * @brief Ros message to primitive class
 */
Primitive toPrimitive(const planning_ros_msgs::Primitive& pr) {
  Vec6f cx, cy, cz;
  for(int i = 0; i < 6; i++) {
    cx(i) = pr.cx[i];
    cy(i) = pr.cy[i];
    cz(i) = pr.cz[i];
  }
  vec_E<Vec6f> cs;
  cs.push_back(cx);
  cs.push_back(cy);
  cs.push_back(cz);

  return Primitive(cs, pr.t);
}

/**
 * @brief Ros message to trajectory class 
 */
Trajectory toTrajectory(const planning_ros_msgs::Trajectory & traj_msg) {
  // Constructor from ros msg
  Trajectory traj;
  traj.taus.push_back(0);
  for(const auto& it: traj_msg.primitives) {
    traj.segs.push_back(toPrimitive(it));
    traj.taus.push_back(traj.taus.back() + it.t);
  }

  if(!traj_msg.lambda.empty()) {
    Lambda l;
    for(int i = 0; i < (int)traj_msg.lambda.size(); i++) {
      LambdaSeg seg;
      seg.a(0) = traj_msg.lambda[i].ca[0];
      seg.a(1) = traj_msg.lambda[i].ca[1];
      seg.a(2) = traj_msg.lambda[i].ca[2];
      seg.a(3) = traj_msg.lambda[i].ca[3];
      seg.ti = traj_msg.lambda[i].ti;
      seg.tf = traj_msg.lambda[i].tf;
      seg.dT = traj_msg.lambda[i].dT;
      l.segs.push_back(seg);
      traj.total_t_ += seg.dT;
    }
    traj.lambda_ = l;
    std::vector<decimal_t> ts;
    for(const auto& tau: traj.taus)
      ts.push_back(traj.lambda_.getT(tau));
    traj.Ts = ts;
  }
  else
    traj.total_t_ = traj.taus.back();
  return traj;
}


