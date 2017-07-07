/**
 * @file primitive_ros_utils.cpp
 * @brief Cpp for interface between primitive classes and ros
 */
#include <ros_utils/primitive_ros_utils.h>

/**
 * @brief Primitive class to primitive ros message
 */
planning_ros_msgs::Primitive toPrimitiveROSMsg(const Primitive& p) {
  planning_ros_msgs::Primitive msg;
  const Vec6f cx = p.traj(0).coeff();
  const Vec6f cy = p.traj(1).coeff();
  const Vec6f cz = p.traj(2).coeff();
  msg.cx.resize(6);
  msg.cy.resize(6);
  msg.cz.resize(6);
  for(int i = 0; i < 6; i++) {
    msg.cx[i] = cx(i);
    msg.cy[i] = cy(i);
    msg.cz[i] = cz(i);
  }
  msg.t = p.t();

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
planning_ros_msgs::Trajectories toTrajectoriesROSMsg(const std::vector<Trajectory>& trajs) {
  planning_ros_msgs::Trajectories msg;
  for(const auto& seg: trajs)
    msg.trajectories.push_back(toTrajectoryROSMsg(seg));
  return msg;
}

/**
 * @brief Ros message to primitive class
 */
Primitive toPrimitive(const planning_ros_msgs::Primitive& p) {
  Vec6f cx, cy, cz;
  for(int i = 0; i < 6; i++) {
    cx(i) = p.cx[i];
    cy(i) = p.cy[i];
    cz(i) = p.cz[i];
  }
  vec_E<Vec6f> cs;
  cs.push_back(cx);
  cs.push_back(cy);
  cs.push_back(cz);

  return Primitive(cs, p.t);
}

/**
 * @brief Ros message to trajectory class 
 */
Trajectory toTrajectory(const planning_ros_msgs::Trajectory & ps) {
  // Constructor from ros msg
  Trajectory traj;
  traj.taus.push_back(0);
  for(const auto& it: ps.primitives) {
    traj.segs.push_back(toPrimitive(it));
    traj.taus.push_back(traj.taus.back() + it.t);
  }

  if(!ps.lambda.empty()) {
    Lambda l;
    for(int i = 0; i < (int)ps.lambda.size(); i++) {
      LambdaSeg seg;
      seg.a(0) = ps.lambda[i].ca[0];
      seg.a(1) = ps.lambda[i].ca[1];
      seg.a(2) = ps.lambda[i].ca[2];
      seg.a(3) = ps.lambda[i].ca[3];
      seg.ti = ps.lambda[i].ti;
      seg.tf = ps.lambda[i].tf;
      seg.dT = ps.lambda[i].dT;
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


