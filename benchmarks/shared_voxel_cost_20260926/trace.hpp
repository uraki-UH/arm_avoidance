#pragma once
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <mutex>
#include <time.h>

// 計測用コピー限定の区間記録。本番バイナリへの組込みなし。
namespace shared_bench {
inline long long steady_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
inline long long cpu_ns() {
  timespec value{}; clock_gettime(CLOCK_THREAD_CPUTIME_ID, &value);
  return value.tv_sec * 1000000000LL + value.tv_nsec;
}
inline void record(const char *kind, long long begin_ns, long long wall_ns, long long cpu_time_ns,
                   long long stamp_ns = 0, std::size_t num = 0) {
  static std::mutex mutex;
  static std::ofstream output(std::getenv("SHARED_BENCH_TRACE"));
  std::lock_guard<std::mutex> lock(mutex);
  output << kind << ',' << begin_ns << ',' << wall_ns/1e6 << ',' << cpu_time_ns/1e6
         << ',' << stamp_ns << ',' << num << '\n';
}
struct scope {
  const char *kind;
  long long begin_ns{steady_ns()}, begin_cpu_ns{cpu_ns()}, stamp_ns{0};
  std::size_t num{0};
  bool is_done{false};
  explicit scope(const char *name) : kind(name) {}
  void finish() {
    if (is_done) {return;}
    const auto wall = steady_ns() - begin_ns, cpu = cpu_ns() - begin_cpu_ns;
    record(kind, begin_ns, wall, cpu, stamp_ns, num); is_done = true;
  }
  ~scope() {finish();}
};
inline bool enable_markers() {
  static const bool enabled = std::getenv("SHARED_BENCH_MARKERS")[0] == '1';
  return enabled;
}
}
