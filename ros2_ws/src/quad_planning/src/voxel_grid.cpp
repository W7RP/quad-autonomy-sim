#include "quad_planning/voxel_grid.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace quad_planning
{

VoxelGrid::VoxelGrid(const GridSpec & spec)
: spec_(spec)
{
  if (spec.nx <= 0 || spec.ny <= 0 || spec.nz <= 0 || !(spec.resolution > 0.0)) {
    throw std::invalid_argument("VoxelGrid: bad spec");
  }
  const auto n = static_cast<std::size_t>(spec.nx) * static_cast<std::size_t>(spec.ny) *
    static_cast<std::size_t>(spec.nz);
  cells_.assign(n, 0);
  scratch_.assign(n, 0);
}

void VoxelGrid::clear() noexcept {std::fill(cells_.begin(), cells_.end(), std::uint8_t{0});}

bool VoxelGrid::index(const Vec3 & p, int & i, int & j, int & k) const noexcept
{
  const Vec3 q = (p - spec_.origin) / spec_.resolution;
  if (!q.allFinite()) {
    return false;
  }
  i = static_cast<int>(std::floor(q.x()));
  j = static_cast<int>(std::floor(q.y()));
  k = static_cast<int>(std::floor(q.z()));
  return i >= 0 && j >= 0 && k >= 0 && i < spec_.nx && j < spec_.ny && k < spec_.nz;
}

void VoxelGrid::mark(const Vec3 & p) noexcept
{
  int i, j, k;
  if (index(p, i, j, k)) {
    cells_[flat(i, j, k)] = 1;
  }
}

void VoxelGrid::merge(const VoxelGrid & other) noexcept
{
  const std::size_t n = std::min(cells_.size(), other.cells_.size());
  for (std::size_t c = 0; c < n; ++c) {
    cells_[c] |= other.cells_[c];
  }
}

bool VoxelGrid::occupied(const Vec3 & p) const noexcept
{
  int i, j, k;
  if (!index(p, i, j, k)) {
    return true;  // outside the grid: never plan through it
  }
  return cells_[flat(i, j, k)] != 0;
}

bool VoxelGrid::segment_free(const Vec3 & a, const Vec3 & b) const noexcept
{
  const double len = (b - a).norm();
  const int steps = std::max(1, static_cast<int>(std::ceil(len / (0.5 * spec_.resolution))));
  for (int s = 0; s <= steps; ++s) {
    if (occupied(a + (b - a) * (static_cast<double>(s) / steps))) {
      return false;
    }
  }
  return true;
}

namespace
{
// One separable pass: out[x] = any(in[x - r .. x + r]) along one axis, with a
// running count so each line costs O(length) regardless of r.
template<typename At>
void dilate_line(int length, int r, At in_at, std::uint8_t * out, std::size_t out_stride)
{
  int count = 0;
  for (int x = 0; x < std::min(r, length); ++x) {
    count += in_at(x);
  }
  for (int x = 0; x < length; ++x) {
    const int add = x + r;
    const int drop = x - r - 1;
    if (add < length) {
      count += in_at(add);
    }
    if (drop >= 0) {
      count -= in_at(drop);
    }
    out[static_cast<std::size_t>(x) * out_stride] = count > 0 ? 1 : 0;
  }
}
}  // namespace

void VoxelGrid::dilate_into(int r, VoxelGrid & out) noexcept
{
  const int nx = spec_.nx, ny = spec_.ny, nz = spec_.nz;
  const std::size_t sx = 1, sy = static_cast<std::size_t>(nx),
    sz = static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny);
  // x pass: cells_ -> scratch_
  for (int k = 0; k < nz; ++k) {
    for (int j = 0; j < ny; ++j) {
      const std::size_t base = flat(0, j, k);
      dilate_line(nx, r, [&](int x) {return static_cast<int>(cells_[base + x * sx]);},
        &scratch_[base], sx);
    }
  }
  // y pass: scratch_ -> out.cells_
  for (int k = 0; k < nz; ++k) {
    for (int i = 0; i < nx; ++i) {
      const std::size_t base = flat(i, 0, k);
      dilate_line(ny, r, [&](int y) {return static_cast<int>(scratch_[base + y * sy]);},
        &out.cells_[base], sy);
    }
  }
  // z pass: out.cells_ -> scratch_ -> out.cells_
  for (int j = 0; j < ny; ++j) {
    for (int i = 0; i < nx; ++i) {
      const std::size_t base = flat(i, j, 0);
      dilate_line(nz, r, [&](int z) {return static_cast<int>(out.cells_[base + z * sz]);},
        &scratch_[base], sz);
    }
  }
  std::copy(scratch_.begin(), scratch_.end(), out.cells_.begin());
}

std::size_t VoxelGrid::count_occupied() const noexcept
{
  return static_cast<std::size_t>(std::count_if(cells_.begin(), cells_.end(),
         [](std::uint8_t c) {return c != 0;}));
}

}  // namespace quad_planning
