#pragma once

#include <algorithm>
#include <atomic>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

namespace robot_sim::common {

// 問い合わせごとの独立出力。worker内部状態は同じworkerだけからのアクセス。
template <typename query_type>
void parallel_queries(std::size_t num_queries, std::size_t num_workers, query_type query) {
  if (num_workers == 0) throw std::invalid_argument("Missing collision workers");
  std::atomic<std::size_t> next_idx{0};
  std::atomic<bool> has_failure{false};
  std::exception_ptr failure;
  std::mutex failure_mutex;
  std::vector<std::thread> workers;
  const auto work = [&](std::size_t worker_idx) {
    try {
      while (!has_failure.load(std::memory_order_relaxed)) {
        const auto idx = next_idx.fetch_add(1, std::memory_order_relaxed);
        if (idx >= num_queries) break;
        query(worker_idx, idx);
      }
    } catch (...) {
      std::lock_guard<std::mutex> guard(failure_mutex);
      if (!failure) failure = std::current_exception();
      has_failure.store(true, std::memory_order_relaxed);
    }
  };
  try {
    for (std::size_t idx = 0; idx < std::min(num_queries, num_workers); ++idx)
      workers.emplace_back(work, idx);
  } catch (...) {
    has_failure.store(true, std::memory_order_relaxed);
    for (auto &worker : workers) worker.join();
    throw;
  }
  for (auto &worker : workers) worker.join();
  if (failure) std::rethrow_exception(failure);
}

}  // robot_sim::common名前空間の終端
