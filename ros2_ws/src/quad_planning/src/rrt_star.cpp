#include "quad_planning/rrt_star.hpp"

#include <algorithm>
#include <limits>

namespace quad_planning
{

RrtStar::RrtStar(RrtConfig config)
: cfg_(std::move(config)), rng_(cfg_.seed)
{
  nodes_.reserve(static_cast<std::size_t>(cfg_.max_nodes));
  near_.reserve(static_cast<std::size_t>(cfg_.max_nodes));
}

std::optional<std::vector<Vec3>> RrtStar::plan(
  const Vec3 & start, const Vec3 & goal, const SegmentFree & free)
{
  const auto t0 = std::chrono::steady_clock::now();
  stats_ = PlanStats{};
  nodes_.clear();
  nodes_.push_back({start, -1, 0.0});
  if (!free(start, start)) {
    return std::nullopt;  // starting inside an obstacle (inflated)
  }

  std::uniform_real_distribution<double> u01(0.0, 1.0);
  std::uniform_real_distribution<double> ux(cfg_.bounds_min.x(), cfg_.bounds_max.x());
  std::uniform_real_distribution<double> uy(cfg_.bounds_min.y(), cfg_.bounds_max.y());
  std::uniform_real_distribution<double> uz(cfg_.bounds_min.z(), cfg_.bounds_max.z());

  int best_goal_node = -1;
  double best_cost = std::numeric_limits<double>::infinity();

  // A direct segment is the optimum: take it without sampling.
  const bool direct = free(start, goal);
  if (direct) {
    nodes_.push_back({goal, 0, (goal - start).norm()});
    best_goal_node = 1;
    best_cost = nodes_[1].cost;
  }

  int it = 0;
  for (; !direct && it < cfg_.max_iterations &&
    static_cast<int>(nodes_.size()) < cfg_.max_nodes; ++it)
  {
    if ((it & 63) == 0 && std::chrono::steady_clock::now() - t0 > cfg_.time_budget) {
      break;
    }
    const Vec3 sample = (u01(rng_) < cfg_.goal_bias) ? goal : Vec3(ux(rng_), uy(rng_), uz(rng_));

    // nearest
    int nearest = 0;
    double best_d = std::numeric_limits<double>::infinity();
    for (int n = 0; n < static_cast<int>(nodes_.size()); ++n) {
      const double d = (nodes_[n].p - sample).squaredNorm();
      if (d < best_d) {
        best_d = d;
        nearest = n;
      }
    }
    // steer
    Vec3 dir = sample - nodes_[nearest].p;
    const double dist = dir.norm();
    if (dist < 1e-6) {
      continue;
    }
    const Vec3 x_new = dist > cfg_.step ? nodes_[nearest].p + dir * (cfg_.step / dist) : sample;
    if (!free(nodes_[nearest].p, x_new)) {
      continue;
    }

    // choose the best parent among near nodes
    near_.clear();
    const double r2 = cfg_.rewire_radius * cfg_.rewire_radius;
    for (int n = 0; n < static_cast<int>(nodes_.size()); ++n) {
      if ((nodes_[n].p - x_new).squaredNorm() <= r2) {
        near_.push_back(n);
      }
    }
    int parent = nearest;
    double cost = nodes_[nearest].cost + (x_new - nodes_[nearest].p).norm();
    for (const int n : near_) {
      const double c = nodes_[n].cost + (x_new - nodes_[n].p).norm();
      if (c < cost && free(nodes_[n].p, x_new)) {
        parent = n;
        cost = c;
      }
    }
    const int id = static_cast<int>(nodes_.size());
    nodes_.push_back({x_new, parent, cost});

    // rewire near nodes through the new one when that is cheaper
    for (const int n : near_) {
      const double c = cost + (nodes_[n].p - x_new).norm();
      if (c < nodes_[n].cost && free(x_new, nodes_[n].p)) {
        nodes_[n].parent = id;
        nodes_[n].cost = c;  // descendants' costs go stale; only used as a heuristic
      }
    }

    // goal connection
    if ((x_new - goal).norm() <= cfg_.goal_tolerance ||
      ((x_new - goal).norm() <= cfg_.step && free(x_new, goal)))
    {
      const double c = cost + (goal - x_new).norm();
      if (c < best_cost) {
        best_cost = c;
        best_goal_node = id;
      }
    }
  }

  stats_.iterations = it;
  stats_.nodes = static_cast<int>(nodes_.size());
  stats_.elapsed_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t0).count();
  if (best_goal_node < 0) {
    return std::nullopt;
  }

  // Walk back; recompute the length from geometry (stored costs of rewired
  // subtrees can be stale).
  std::vector<Vec3> path;
  for (int n = best_goal_node; n >= 0; n = nodes_[n].parent) {
    path.push_back(nodes_[n].p);
  }
  std::reverse(path.begin(), path.end());
  if ((path.back() - goal).norm() > 1e-9) {
    path.push_back(goal);
  }
  stats_.found = true;
  stats_.cost = path_length(path);
  return path;
}

std::vector<Vec3> shortcut(const std::vector<Vec3> & path, const SegmentFree & free)
{
  if (path.size() <= 2) {
    return path;
  }
  std::vector<Vec3> out{path.front()};
  std::size_t i = 0;
  while (i < path.size() - 1) {
    std::size_t j = path.size() - 1;
    while (j > i + 1 && !free(path[i], path[j])) {
      --j;
    }
    out.push_back(path[j]);
    i = j;
  }
  return out;
}

double path_length(const std::vector<Vec3> & path) noexcept
{
  double len = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    len += (path[i] - path[i - 1]).norm();
  }
  return len;
}

}  // namespace quad_planning
