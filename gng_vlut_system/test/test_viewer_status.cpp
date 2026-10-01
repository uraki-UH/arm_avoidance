#include <gtest/gtest.h>
#include <limits>
#include "core/common/viewer_status.hpp"

using robot_sim::common::viewer_status_snapshot;

TEST(viewer_status, counts_and_stage_times) {
  viewer_status_snapshot status;
  const auto now = viewer_status_snapshot::clock::now();
  status.update(0, {120938, 2.01}, now);
  status.update(1, {821, 340, 12, 0.08}, now);
  status.update(2, {10384, 48, 369, 0.11}, now);
  EXPECT_EQ(status.format(now),
      "PCl: 120938 (2.01 ms) | Vxl: Env 821 Self 340 Dup 12 (0.08 ms) | Emap: Safe 10384 Coll 48 Dang 369 (0.11 ms)");
}

TEST(viewer_status, missing_stale_and_recovery) {
  viewer_status_snapshot status;
  const auto now = viewer_status_snapshot::clock::now();
  const auto empty = status.format(now);
  EXPECT_EQ(empty, "PCl: -- (-- ms) | Vxl: Env -- Self -- Dup -- (-- ms) | Emap: Safe -- Coll -- Dang -- (-- ms)");
  status.update(0, {0, 0}, now);
  EXPECT_EQ(status.format(now).find("PCl: 0 (0.00 ms)"), 0U);
  EXPECT_EQ(status.format(now + std::chrono::seconds(4)), empty);
  status.update(0, {3, 1}, now + std::chrono::seconds(4));
  EXPECT_EQ(status.format(now + std::chrono::seconds(4)).find("PCl: 3 (1.00 ms)"), 0U);
}

TEST(viewer_status, invalid_statistics_do_not_refresh) {
  viewer_status_snapshot status;
  const auto now = viewer_status_snapshot::clock::now();
  const auto empty = status.format(now);
  status.update(0, {1});
  status.update(0, {1, -1});
  status.update(0, {1.5, 1});
  status.update(1, {1, 2, 3, std::numeric_limits<double>::quiet_NaN()});
  status.update(9, {1, 2});
  EXPECT_EQ(status.format(now), empty);
}

TEST(viewer_status, independent_stage_freshness) {
  viewer_status_snapshot status;
  const auto now = viewer_status_snapshot::clock::now();
  status.update(0, {4, 1}, now - std::chrono::seconds(4));
  status.update(2, {8, 0, 1, 0.01}, now);
  EXPECT_EQ(status.format(now), "PCl: -- (-- ms) | Vxl: Env -- Self -- Dup -- (-- ms) | Emap: Safe 8 Coll 0 Dang 1 (0.01 ms)");
}
