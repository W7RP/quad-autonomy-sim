// Runtime heap-allocation probe for the estimator's hot path.
//
// alloc_probe.cpp replaces the global operator new (in the node executable
// only) and counts every allocation made:
//   * inside a HotPathScope (our callback bodies), and
//   * anywhere on a thread that called track_this_thread() (our callbacks PLUS
//     rclcpp's take/deserialise path and the executor running around them).
// Both counters are published as diagnostics, so "no allocation in the hot
// path" is a measured number rather than a claim. Counting is two relaxed
// atomic increments guarded by thread_local flags: cheap enough to leave on.
//
// Limitation: memory obtained with malloc() directly (not operator new) is not
// counted. Eigen fixed-size types never allocate; the unit tests separately run
// the filter with Eigen's own runtime malloc check armed.
#pragma once

#include <cstdint>

namespace quad_estimation::alloc_probe
{

void track_this_thread() noexcept;

class HotPathScope
{
public:
  HotPathScope() noexcept;
  ~HotPathScope();
  HotPathScope(const HotPathScope &) = delete;
  HotPathScope & operator=(const HotPathScope &) = delete;
};

[[nodiscard]] std::uint64_t hot_path_allocations() noexcept;
[[nodiscard]] std::uint64_t tracked_thread_allocations() noexcept;

}  // namespace quad_estimation::alloc_probe
