// RRT* in 3D with a fixed node pool, plus greedy shortcut smoothing.
//
// Planning is anytime and bounded: at most max_nodes nodes (allocated once, in
// the constructor) and at most max_iterations samples or time_budget, whichever
// comes first. It keeps improving the best path until the budget runs out.
// Nearest and near-neighbour queries are linear scans: with a few thousand
// nodes that is fast enough, and simple to bound.
//
// Collision checking is the caller's: any callable bool(const Vec3&, const Vec3&)
// reporting whether a straight segment is free.
#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <optional>
#include <random>
#include <vector>

#include <Eigen/Core>

namespace quad_planning
{

using Vec3 = Eigen::Vector3d;
using SegmentFree = std::function<bool(const Vec3 &, const Vec3 &)>;

struct RrtConfig
{
  Vec3 bounds_min{-10.0, -10.0, 0.5};  // sampling box
  Vec3 bounds_max{10.0, 10.0, 3.0};
  double step{1.0};             // [m] max extension per iteration
  double goal_bias{0.1};        // probability of sampling the goal
  double rewire_radius{2.5};    // [m]
  double goal_tolerance{0.3};   // [m] a node this close to the goal "reaches" it
  int max_nodes{4000};
  int max_iterations{20000};
  std::chrono::milliseconds time_budget{400};
  std::uint32_t seed{1};
};

struct PlanStats
{
  int iterations{0};
  int nodes{0};
  double cost{0.0};             // path length [m] before smoothing
  double elapsed_ms{0.0};
  bool found{false};
};

class RrtStar
{
public:
  explicit RrtStar(RrtConfig config);

  // Path from start to goal (both included), or std::nullopt if none was found
  // within the budget. The start itself must be free.
  [[nodiscard]] std::optional<std::vector<Vec3>> plan(
    const Vec3 & start, const Vec3 & goal, const SegmentFree & free);

  [[nodiscard]] const PlanStats & stats() const noexcept {return stats_;}
  [[nodiscard]] const RrtConfig & config() const noexcept {return cfg_;}
  void reseed(std::uint32_t seed) {rng_.seed(seed);}

private:
  struct Node
  {
    Vec3 p;
    int parent;
    double cost;
  };

  RrtConfig cfg_;
  std::vector<Node> nodes_;  // capacity max_nodes, reserved once
  std::vector<int> near_;    // near-neighbour scratch, reserved once
  std::mt19937 rng_;
  PlanStats stats_;
};

// Greedy "string pulling": from each kept waypoint, jump to the farthest later
// waypoint still reachable by a free straight segment. Keeps start and goal.
[[nodiscard]] std::vector<Vec3> shortcut(const std::vector<Vec3> & path, const SegmentFree & free);

[[nodiscard]] double path_length(const std::vector<Vec3> & path) noexcept;

}  // namespace quad_planning
