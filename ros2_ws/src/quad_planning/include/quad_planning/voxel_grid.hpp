// Fixed-size 3D occupancy grid for planning.
//
// Memory is allocated once, at construction; marking, clearing, inflation and
// queries never allocate. Everything outside the grid's bounds counts as
// occupied: the planner must never route through space it cannot represent.
//
// Frame: whatever the caller uses consistently (the planner uses `odom`, ENU,
// metres). ROS-free so it can be unit-tested.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>

namespace quad_planning
{

using Vec3 = Eigen::Vector3d;

struct GridSpec
{
  Vec3 origin{Vec3::Zero()};  // world position of the corner of cell (0, 0, 0)
  double resolution{0.1};     // [m]
  int nx{0};
  int ny{0};
  int nz{0};
};

class VoxelGrid
{
public:
  explicit VoxelGrid(const GridSpec & spec);

  [[nodiscard]] const GridSpec & spec() const noexcept {return spec_;}
  [[nodiscard]] std::size_t size() const noexcept {return cells_.size();}

  void clear() noexcept;
  // Mark the cell containing p as occupied. Points outside the grid are ignored.
  void mark(const Vec3 & p) noexcept;
  // Cell-wise OR with another grid of the same spec.
  void merge(const VoxelGrid & other) noexcept;

  // Occupied (or outside the grid)?
  [[nodiscard]] bool occupied(const Vec3 & p) const noexcept;
  // Is the straight segment a-b free? Sampled at half the resolution, so it
  // cannot step over a single occupied cell.
  [[nodiscard]] bool segment_free(const Vec3 & a, const Vec3 & b) const noexcept;

  // out = this grid dilated by a box of half-width r cells in x, y and z (a
  // conservative stand-in for a sphere of radius r * resolution). O(cells):
  // separable, one sliding-window pass per axis. `out` must have the same spec.
  void dilate_into(int r, VoxelGrid & out) noexcept;

  [[nodiscard]] std::size_t count_occupied() const noexcept;

private:
  [[nodiscard]] bool index(const Vec3 & p, int & i, int & j, int & k) const noexcept;
  [[nodiscard]] std::size_t flat(int i, int j, int k) const noexcept
  {
    return (static_cast<std::size_t>(k) * static_cast<std::size_t>(spec_.ny) +
           static_cast<std::size_t>(j)) * static_cast<std::size_t>(spec_.nx) +
           static_cast<std::size_t>(i);
  }

  GridSpec spec_;
  std::vector<std::uint8_t> cells_;
  std::vector<std::uint8_t> scratch_;  // dilation intermediate, allocated once
};

}  // namespace quad_planning
