// Fixed-capacity FIFO ring buffer. Storage is an inline std::array: constructing
// one is the only time memory is touched, push/pop never allocate, and every
// operation is O(1) (iteration is O(N) with N the compile-time capacity).
//
// Not thread-safe by design: each ring is owned by the estimator's single
// sensor thread (see docs/phase2_state_estimation.md, "Threads").
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace quad_estimation
{

template<typename T, std::size_t N>
class FixedRing
{
  static_assert(N > 0, "capacity must be positive");

public:
  // Push to the back. When full, the oldest element is overwritten and
  // overwritten() is incremented, so data loss is visible in diagnostics.
  void push(const T & value) noexcept
  {
    if (size_ == N) {
      head_ = (head_ + 1) % N;
      --size_;
      ++overwritten_;
    }
    data_[(head_ + size_) % N] = value;
    ++size_;
  }

  [[nodiscard]] const T & front() const noexcept {return data_[head_];}
  void pop_front() noexcept
  {
    if (size_ > 0) {
      head_ = (head_ + 1) % N;
      --size_;
    }
  }

  // i = 0 is the oldest element.
  [[nodiscard]] const T & operator[](std::size_t i) const noexcept {return data_[(head_ + i) % N];}
  [[nodiscard]] std::size_t size() const noexcept {return size_;}
  [[nodiscard]] bool empty() const noexcept {return size_ == 0;}
  static constexpr std::size_t capacity() noexcept {return N;}
  void clear() noexcept {head_ = 0; size_ = 0;}
  [[nodiscard]] std::uint64_t overwritten() const noexcept {return overwritten_;}

private:
  std::array<T, N> data_{};
  std::size_t head_{0};
  std::size_t size_{0};
  std::uint64_t overwritten_{0};
};

}  // namespace quad_estimation
