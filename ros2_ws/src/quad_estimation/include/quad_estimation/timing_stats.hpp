// Lock-free execution-time statistics for one callback: written by the sensor
// thread, read by the diagnostics thread. Fixed histogram, relaxed atomics:
// recording is O(1) with no allocation and never blocks.
#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>

namespace quad_estimation
{

class TimingStats
{
public:
  static constexpr std::int64_t kBinUs = 5;
  static constexpr std::size_t kBins = 400;  // 0 .. 2 ms in 5 us bins, last bin = overflow

  explicit TimingStats(std::int64_t budget_us = 1000) noexcept
  : budget_us_(budget_us) {}

  void record(std::int64_t us) noexcept
  {
    count_.fetch_add(1, std::memory_order_relaxed);
    sum_us_.fetch_add(static_cast<std::uint64_t>(us), std::memory_order_relaxed);
    std::int64_t prev = max_us_.load(std::memory_order_relaxed);
    while (us > prev && !max_us_.compare_exchange_weak(prev, us, std::memory_order_relaxed)) {
    }
    if (us > budget_us_) {
      overruns_.fetch_add(1, std::memory_order_relaxed);
    }
    const auto bin = static_cast<std::size_t>(std::clamp<std::int64_t>(us / kBinUs, 0, kBins - 1));
    hist_[bin].fetch_add(1, std::memory_order_relaxed);
  }

  struct Snapshot
  {
    std::uint64_t count{};
    double mean_us{};
    std::int64_t max_us{};
    std::int64_t p99_us{};   // upper edge of the bin holding the 99th percentile
    std::uint64_t overruns{};
    std::int64_t budget_us{};
  };

  [[nodiscard]] Snapshot snapshot() const noexcept
  {
    Snapshot s;
    s.count = count_.load(std::memory_order_relaxed);
    s.max_us = max_us_.load(std::memory_order_relaxed);
    s.overruns = overruns_.load(std::memory_order_relaxed);
    s.budget_us = budget_us_;
    s.mean_us = s.count ? static_cast<double>(sum_us_.load(std::memory_order_relaxed)) /
      static_cast<double>(s.count) : 0.0;
    const std::uint64_t target = s.count - s.count / 100;  // 99th percentile rank
    std::uint64_t seen = 0;
    for (std::size_t i = 0; i < kBins; ++i) {
      seen += hist_[i].load(std::memory_order_relaxed);
      if (seen >= target && s.count > 0) {
        s.p99_us = static_cast<std::int64_t>(i + 1) * kBinUs;
        break;
      }
    }
    return s;
  }

private:
  std::int64_t budget_us_;
  std::atomic<std::uint64_t> count_{0};
  std::atomic<std::uint64_t> sum_us_{0};
  std::atomic<std::int64_t> max_us_{0};
  std::atomic<std::uint64_t> overruns_{0};
  std::array<std::atomic<std::uint64_t>, kBins> hist_{};
};

}  // namespace quad_estimation
