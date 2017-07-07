#include <planner/bfs.h>

using namespace MPL;

Graph::Graph() { 
}

void Graph::addNode(const Waypoint& node) {
  nodes_.push_back(node);
}

void Graph::addEdge(const Primitive& pr) {
  edges_.push_back(pr);
}

void Graph::clear() {
  nodes_.clear();
  edges_.clear();
}

vec_Vec3f Graph::ps() {
  vec_Vec3f ps;
  for(const auto& it: nodes_) {
    ps.push_back(it.pos);
    ps.back()(2) += it.gcost / 20.;
  }
  return ps;
}

std::vector<Waypoint> Graph::nodes() {
  return nodes_;
}

std::vector<Primitive> Graph::edges() {
  return edges_;
}

void Graph::info() {
  printf("Number of nodes is %zu\n", nodes_.size());
  printf("Number of edges is %zu\n", edges_.size());
}

BFS::BFS() {
  planner_verbose_ = false;

}

void BFS::setMapUtil(std::shared_ptr<VoxelMapUtil> map_util) {
  map_util_ = map_util;

  decimal_t r = 4.0;
  decimal_t h = 0;
  decimal_t res = map_util_->getRes();
  int dilate_xy = std::ceil(r / res);
  int dilate_z = std::ceil(h / res); 
  collision_neighbors_.clear();
  Vec3i n;
  for (n(0) = -dilate_xy; n(0) <= dilate_xy; n(0)++) {
    for (n(1) = -dilate_xy; n(1) <= dilate_xy; n(1)++) {
      for (n(2) = -dilate_z; n(2) <= dilate_z; n(2)++) {
        decimal_t d = res * n.topRows(2).norm() - r;
        // if (d >= -0.1 && d <= res_ * 0.71)
        if (d <= res * 0.71)
          collision_neighbors_.push_back(n);
      }
    }
  }
}

void BFS::setDt(decimal_t dt) {
  dt_ = dt;
}

void BFS::setDiscretization(decimal_t u_max, int n, bool use_3d) {
  decimal_t du = u_max / n;
  U_.clear();
  if(use_3d) {
    for(decimal_t dx = -u_max; dx <= u_max; dx += du )
      for(decimal_t dy = -u_max; dy <= u_max; dy += du )
        for(decimal_t dz = -0.5; dz <= 0.5; dz += 0.5 )
          U_.push_back(Vec3f(dx, dy, dz));
  }
  else{
    for(decimal_t dx = -u_max; dx <= u_max; dx += du )
      for(decimal_t dy = -u_max; dy <= u_max; dy += du )
        U_.push_back(Vec3f(dx, dy, 0));
  }
}

bool BFS::isFree(const Primitive& p) {
  std::vector<Waypoint> pts = p.sample(10);
  for(const auto& pt: pts) {
    Vec3i pn = map_util_->floatToInt(pt.pos);
    if(map_util_->isOccupied(pn))
      return false;
  }

  return true;
}

void BFS::createGraph(const Waypoint& start, const Waypoint& goal, int num) {
  decimal_t gamma_ = 1.0;
  graph_.clear();
  std::stack<Waypoint> stack;
  stack.push(start);
  graph_.addNode(start);

  while(!stack.empty()) {
    Waypoint curr = stack.top();
    stack.pop();

    for(const auto& it: U_) {
      Primitive pr(curr, it, dt_);
      if(isFree(pr) && pr.valid_vel(v_max_)) {
        Waypoint next = pr.evaluate(dt_);
        next.use_pos = curr.use_pos;
        next.use_vel = curr.use_vel;
        next.use_acc = curr.use_acc;
        next.n = curr.n + 1;
        next.parent = std::make_shared<Waypoint>(curr);
        if((next.pos != curr.pos ||
            next.vel != curr.vel ||
            next.acc != curr.acc) &&
            next.n <= num) {
          int iter = curr.use_acc ? 2:1;
          decimal_t edge_cost = pr.J(iter) + w_ * dt_ ;
          next.cost = curr.cost + std::pow(gamma_, next.n) * edge_cost;
          next.gcost = next.cost + heuristic(next, goal) + w_collision_ * collision(next);
          stack.push(next);
          graph_.addNode(next);
          graph_.addEdge(pr);
        }
      }
    }
  }
  graph_.info();
}

decimal_t BFS::heuristic(const Waypoint& node, const Waypoint& goal_node) {
  //return 0;
  //return w_*(node.pos - goal_node_.pos).lpNorm<Eigen::Infinity>() / v_max_;
  const Vec3f p0 = node.pos;
  const Vec3f p1 = goal_node.pos;
  const Vec3f v0 = node.vel;
  const Vec3f v1 = goal_node.vel;
  decimal_t c1 = -36*(p1-p0).dot(p1-p0);
  decimal_t c2 = 24*(v0+v1).dot(p1-p0);
  decimal_t c3 = -4*(v0.dot(v0)+v0.dot(v1)+v1.dot(v1));
  decimal_t c4 = 0;
  decimal_t c5 = w_;

  std::vector<decimal_t> ts = quartic(c5, c4, c3, c2, c1);
  decimal_t t_bar = (node.pos - goal_node.pos).lpNorm<Eigen::Infinity>() / v_max_;
  ts.push_back(t_bar);
  decimal_t cost = 1000000;
  for(auto t: ts) {
    if(t < t_bar)
      continue;
    decimal_t c = -c1/3/t/t/t-c2/2/t/t-c3/t+w_*t;
    if(c < cost)
      cost = c;
    //printf("t: %f, cost: %f\n",t, cost);
  }
  //printf("-----------\n");
  if(ts.empty())
    printf("wrong! no root fond!\n");

  return cost;
}

decimal_t BFS::collision(const Waypoint& node) {
  decimal_t c = 0;
  decimal_t min_tth = std::numeric_limits<decimal_t>::max();
  Vec3i pn = map_util_->floatToInt(node.pos);
  for(const auto &it: collision_neighbors_) {
    if(map_util_->isOccupied(it + pn)) {
      decimal_t dist = it.cast<decimal_t>().norm();
      decimal_t tth = dist / (node.vel.dot(it.cast<decimal_t>())/dist);
      if(tth >= 0 && tth < min_tth) 
        min_tth = tth;
    }
  }

  decimal_t w_tth = 1.0;
  if(min_tth < std::numeric_limits<decimal_t>::max()) {
    c = 1. / (1 + exp(-w_tth/min_tth)) - 0.5;
    //printf("tth: %f, c: %f\n", min_tth, c);
  }

  return c > 0 ? c: 0;
}

std::vector<Primitive> BFS::recoverTraj(int num) {
  decimal_t min_cost = std::numeric_limits<decimal_t>::max();
  int min_id = -1;
  std::vector<Waypoint> nodes = graph_.nodes();
  for(unsigned int i = 1; i < nodes.size(); i ++) {
    if(nodes[i].gcost < min_cost && nodes[i].n == num) {
      min_cost = nodes[i].gcost;
      min_id = i;
    }
  }

  std::vector<Primitive> prs;
  Waypoint curr = nodes[min_id];
  while(curr.parent != NULL) {
    curr.parent->print();
    curr.print();
    Primitive pr(*curr.parent, curr, dt_);
    prs.push_back(pr);
    curr = *curr.parent;
  }

  std::reverse(prs.begin(), prs.end());
  return prs;
}

vec_Vec3f BFS::getPs() {
  return graph_.ps();
}

std::vector<Primitive> BFS::getPrimitives() {
  return graph_.edges();
}
