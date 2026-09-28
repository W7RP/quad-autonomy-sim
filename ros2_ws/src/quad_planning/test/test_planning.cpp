#include <gtest/gtest.h>

#include <random>

#include "quad_planning/rrt_star.hpp"
#include "quad_planning/voxel_grid.hpp"

using namespace quad_planning;

namespace
{
GridSpec spec_20m()
{
  GridSpec s;
  s.origin = Vec3(-10.0, -10.0, 0.0);
  s.resolution = 0.1;
  s.nx = 200;
  s.ny = 200;
  s.nz = 40;
  return s;
}

// A wall at x = 0 across the whole y range from z 0..4, except a 2 m doorway
// at y in [3, 5].
void wall_with_door(VoxelGrid & g)
{
  for (double y = -10.0; y < 10.0; y += 0.05) {
    if (y > 3.0 && y < 5.0) {
      continue;
    }
    for (double z = 0.0; z < 4.0; z += 0.05) {
      g.mark({0.0, y, z});
      g.mark({0.05, y, z});
    }
  }
}
}  // namespace

TEST(VoxelGrid, MarkQueryAndOutOfBounds)
{
  VoxelGrid g(spec_20m());
  EXPECT_FALSE(g.occupied({1.0, 1.0, 1.0}));
  g.mark({1.0, 1.0, 1.0});
  EXPECT_TRUE(g.occupied({1.03, 1.07, 1.02}));   // same 10 cm cell
  EXPECT_FALSE(g.occupied({1.15, 1.0, 1.0}));    // neighbour cell
  EXPECT_TRUE(g.occupied({50.0, 0.0, 1.0}));     // outside the grid counts as occupied
  EXPECT_EQ(g.count_occupied(), 1U);
  g.clear();
  EXPECT_EQ(g.count_occupied(), 0U);
}

TEST(VoxelGrid, SegmentCannotStepOverOneCell)
{
  VoxelGrid g(spec_20m());
  g.mark({0.0, 0.0, 1.0});
  EXPECT_FALSE(g.segment_free({-3.0, 0.02, 1.02}, {3.0, 0.02, 1.02}));
  EXPECT_TRUE(g.segment_free({-3.0, 0.5, 1.0}, {3.0, 0.5, 1.0}));
}

TEST(VoxelGrid, DilationMatchesBruteForce)
{
  GridSpec s;
  s.resolution = 1.0;
  s.nx = 12;
  s.ny = 9;
  s.nz = 7;
  VoxelGrid g(s), out(s);
  std::mt19937 rng(3);
  std::uniform_int_distribution<int> ux(0, 11), uy(0, 8), uz(0, 6);
  std::vector<Vec3> pts;
  for (int n = 0; n < 6; ++n) {
    pts.emplace_back(ux(rng) + 0.5, uy(rng) + 0.5, uz(rng) + 0.5);
    g.mark(pts.back());
  }
  const int r = 2;
  g.dilate_into(r, out);
  for (int k = 0; k < 7; ++k) {
    for (int j = 0; j < 9; ++j) {
      for (int i = 0; i < 12; ++i) {
        bool expect = false;
        for (const auto & p : pts) {
          expect |= std::abs(i - static_cast<int>(p.x())) <= r &&
            std::abs(j - static_cast<int>(p.y())) <= r && std::abs(k - static_cast<int>(p.z())) <= r;
        }
        ASSERT_EQ(out.occupied({i + 0.5, j + 0.5, k + 0.5}), expect) << i << "," << j << "," << k;
      }
    }
  }
}

TEST(RrtStar, FindsTheDoorAndThePathIsFree)
{
  VoxelGrid raw(spec_20m()), inflated(spec_20m());
  wall_with_door(raw);
  raw.dilate_into(3, inflated);  // 0.3 m inflation
  const auto free = [&](const Vec3 & a, const Vec3 & b) {return inflated.segment_free(a, b);};

  RrtConfig cfg;
  cfg.bounds_min = Vec3(-9.0, -9.0, 1.0);
  cfg.bounds_max = Vec3(9.0, 9.0, 2.5);
  cfg.time_budget = std::chrono::milliseconds(2000);
  RrtStar rrt(cfg);
  const Vec3 start(-5.0, -5.0, 1.5), goal(5.0, -5.0, 1.5);
  const auto path = rrt.plan(start, goal, free);
  ASSERT_TRUE(path.has_value());
  EXPECT_LT((path->front() - start).norm(), 1e-9);
  EXPECT_LT((path->back() - goal).norm(), 1e-9);
  bool through_door = false;
  for (std::size_t i = 1; i < path->size(); ++i) {
    ASSERT_TRUE(free((*path)[i - 1], (*path)[i]));
    through_door |= ((*path)[i - 1].x() < 0) != ((*path)[i].x() < 0);
  }
  EXPECT_TRUE(through_door);
  // The straight line is 10 m through the wall; via the door it is >= 2 * hypot(5, 8).
  EXPECT_GT(path_length(*path), 2 * std::hypot(5.0, 8.0) - 0.5);

  const auto smooth = shortcut(*path, free);
  EXPECT_LE(smooth.size(), path->size());
  EXPECT_LE(path_length(smooth), path_length(*path) + 1e-9);
  for (std::size_t i = 1; i < smooth.size(); ++i) {
    EXPECT_TRUE(free(smooth[i - 1], smooth[i]));
  }
}

TEST(RrtStar, DirectLineWhenFree)
{
  VoxelGrid g(spec_20m());
  const auto free = [&](const Vec3 & a, const Vec3 & b) {return g.segment_free(a, b);};
  RrtStar rrt(RrtConfig{});
  const auto path = rrt.plan({-2, 0, 1.5}, {3, 1, 1.5}, free);
  ASSERT_TRUE(path.has_value());
  EXPECT_EQ(path->size(), 2U);
  EXPECT_EQ(rrt.stats().iterations, 0);
}

TEST(RrtStar, UnreachableGoalFailsWithinBudget)
{
  VoxelGrid raw(spec_20m());
  wall_with_door(raw);
  for (double y = 2.9; y < 5.1; y += 0.05) {  // close the door
    for (double z = 0.0; z < 4.0; z += 0.05) {
      raw.mark({0.0, y, z});
    }
  }
  const auto free = [&](const Vec3 & a, const Vec3 & b) {return raw.segment_free(a, b);};
  RrtConfig cfg;
  cfg.bounds_min = Vec3(-9.0, -9.0, 1.0);
  cfg.bounds_max = Vec3(9.0, 9.0, 2.5);
  cfg.time_budget = std::chrono::milliseconds(150);
  RrtStar rrt(cfg);
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_FALSE(rrt.plan({-5, -5, 1.5}, {5, -5, 1.5}, free).has_value());
  EXPECT_LT(std::chrono::steady_clock::now() - t0, std::chrono::milliseconds(1000));
  EXPECT_FALSE(rrt.stats().found);
}

TEST(RrtStar, DeterministicForASeed)
{
  VoxelGrid raw(spec_20m());
  wall_with_door(raw);
  const auto free = [&](const Vec3 & a, const Vec3 & b) {return raw.segment_free(a, b);};
  RrtConfig cfg;
  cfg.bounds_min = Vec3(-9.0, -9.0, 1.0);
  cfg.bounds_max = Vec3(9.0, 9.0, 2.5);
  cfg.max_iterations = 3000;
  cfg.time_budget = std::chrono::milliseconds(60000);  // iteration-bound, not time-bound
  RrtStar a(cfg), b(cfg);
  const auto pa = a.plan({-5, -5, 1.5}, {5, -5, 1.5}, free);
  const auto pb = b.plan({-5, -5, 1.5}, {5, -5, 1.5}, free);
  ASSERT_TRUE(pa && pb);
  ASSERT_EQ(pa->size(), pb->size());
  for (std::size_t i = 0; i < pa->size(); ++i) {
    EXPECT_LT(((*pa)[i] - (*pb)[i]).norm(), 1e-12);
  }
}
