// Global operator new replacement that counts allocations (see alloc_probe.hpp).
// Linked into the node executable only; libraries and tests use the defaults.
#include "quad_estimation/alloc_probe.hpp"

#include <atomic>
#include <cstdlib>
#include <new>

namespace
{
// Constant-initialised thread_locals: safe to touch from operator new even
// before any dynamic initialisation has run.
thread_local bool t_tracked = false;
thread_local int t_hot_depth = 0;

std::atomic<std::uint64_t> g_hot{0};
std::atomic<std::uint64_t> g_tracked{0};

inline void count() noexcept
{
  if (t_hot_depth > 0) {
    g_hot.fetch_add(1, std::memory_order_relaxed);
  }
  if (t_tracked) {
    g_tracked.fetch_add(1, std::memory_order_relaxed);
  }
}

void * allocate(std::size_t n)
{
  count();
  if (void * p = std::malloc(n == 0 ? 1 : n)) {
    return p;
  }
  throw std::bad_alloc();
}

void * allocate_aligned(std::size_t n, std::align_val_t al)
{
  count();
  const auto a = static_cast<std::size_t>(al);
  const std::size_t rounded = ((n == 0 ? 1 : n) + a - 1) / a * a;  // aligned_alloc contract
  if (void * p = std::aligned_alloc(a, rounded)) {
    return p;
  }
  throw std::bad_alloc();
}
}  // namespace

namespace quad_estimation::alloc_probe
{
void track_this_thread() noexcept {t_tracked = true;}
HotPathScope::HotPathScope() noexcept {++t_hot_depth;}
HotPathScope::~HotPathScope() {--t_hot_depth;}
std::uint64_t hot_path_allocations() noexcept {return g_hot.load(std::memory_order_relaxed);}
std::uint64_t tracked_thread_allocations() noexcept
{
  return g_tracked.load(std::memory_order_relaxed);
}
}  // namespace quad_estimation::alloc_probe

// libstdc++ implements new[], nothrow and sized variants in terms of these two,
// and its default deletes call free(), which matches malloc/aligned_alloc.
void * operator new(std::size_t n) {return allocate(n);}
void * operator new(std::size_t n, std::align_val_t al) {return allocate_aligned(n, al);}
void operator delete(void * p) noexcept {std::free(p);}
void operator delete(void * p, std::size_t) noexcept {std::free(p);}
void operator delete(void * p, std::align_val_t) noexcept {std::free(p);}
void operator delete(void * p, std::size_t, std::align_val_t) noexcept {std::free(p);}
