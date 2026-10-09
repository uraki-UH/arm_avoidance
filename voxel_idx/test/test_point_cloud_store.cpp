#include <point_cloud_store.hpp>
#include <gtest/gtest.h>
#include <map>
#include <random>
#include <thread>
#include <tuple>

using namespace voxel_idx;

TEST(point_cloud_store, registration_queries_share_only_identical_grid_conditions)
{
  auto channel = shared_point_frames("registration_conditions");
  point_registration_spec spec;
  spec.target_frame = "robot";
  const auto query = channel->registration_query(spec);
  EXPECT_EQ(query, channel->registration_query(spec));
  auto changed = spec; changed.size = .2f; changed.num_cells = {{10, 10, 10}};
  EXPECT_NE(query, channel->registration_query(changed));
  changed = spec; changed.target_frame = "world";
  EXPECT_NE(query, channel->registration_query(changed));
  changed = spec; changed.min_pos[0] = -2; changed.num_cells[0] = 30;
  EXPECT_NE(query, channel->registration_query(changed));
  changed = spec; changed.num_cells[0] = 21;
  EXPECT_THROW(channel->registration_query(changed), std::invalid_argument);
  changed = spec; changed.size = NAN;
  EXPECT_THROW(channel->registration_query(changed), std::invalid_argument);
}

TEST(point_cloud_store, registration_once_per_source_point_across_selected_sets)
{
  point_registration_spec spec; spec.target_frame = "robot";
  point_registration_query query(spec);
  auto frame = std::make_shared<const point_frame>();
  const std::vector<float> xyz{.01f, 0, 0, .02f, 0, 0};
  const std::vector<std::uint32_t> indices{2, 0};
  const auto first = query.read(frame, 4, indices, xyz.data());
  EXPECT_EQ(query.num_registered_points(), 2U);
  EXPECT_EQ(first, query.read(frame, 4, indices, xyz.data()));
  EXPECT_EQ(query.num_registered_points(), 2U);
  const auto saved = first->points;
  const std::vector<float> more_xyz{.02f, 0, 0, -.4f, 0, 0, .01f, 0, 0};
  const auto second = query.read(frame, 4, {0, 3, 2}, more_xyz.data());
  EXPECT_EQ(query.num_registered_points(), 3U);
  EXPECT_EQ(first->points, saved);
  ASSERT_EQ(second->points.size(), 3U);
  EXPECT_EQ(static_cast<std::uint32_t>(second->points[0]), 1U);
  EXPECT_EQ(static_cast<std::uint32_t>(second->points[1]), 0U);
  EXPECT_EQ(static_cast<std::uint32_t>(second->points[2]), 2U);
  const auto next_frame = std::make_shared<const point_frame>();
  const float moved[]{.8f, 0, 0};
  const auto third = query.read(next_frame, 4, {2}, moved);
  EXPECT_EQ(query.num_registered_points(), 1U);
  EXPECT_NE(third->points.front() >> 32, first->points.front() >> 32);
  EXPECT_EQ(first->points, saved);
}

TEST(point_cloud_store, registration_float_boundaries_and_stable_input_order)
{
  point_registration_spec spec; spec.target_frame = "robot";
  point_registration_query query(spec);
  std::vector<float> points;
  std::vector<std::uint32_t> indices;
  std::vector<std::uint64_t> expected;
  std::mt19937 random(41); std::uniform_real_distribution<float> coord(-1.2f, 1.2f);
  for (std::uint32_t idx = 0; idx < 10003; ++idx) {
    float p[3]{coord(random), coord(random), coord(random)};
    if (idx < 3) {p[0] = idx == 0 ? -1.f : 1.f; p[1] = p[2] = idx == 2 ? 1.f : 0.f;}
    indices.push_back(idx); points.insert(points.end(), p, p + 3);
    if (p[0] < -1 || p[0] > 1 || p[1] < -1 || p[1] > 1 || p[2] < -1 || p[2] > 1) {continue;}
    const auto x = static_cast<std::uint32_t>((p[0] + 1.f) * (1.f / spec.size));
    const auto y = static_cast<std::uint32_t>((p[1] + 1.f) * (1.f / spec.size));
    const auto z = static_cast<std::uint32_t>((p[2] + 1.f) * (1.f / spec.size));
    const auto cell_idx = x + 20 * y + 400 * z;
    if (cell_idx < 8000) {expected.push_back((std::uint64_t{cell_idx} << 32) | idx);}
  }
  std::sort(expected.begin(), expected.end());
  const auto frame = std::make_shared<const point_frame>();
  const auto result = query.read(frame, indices.size(), indices, points.data());
  EXPECT_EQ(result->points, expected);
  EXPECT_EQ(query.num_registered_points(), indices.size());
}

TEST(point_cloud_store, registration_invalid_input_empty_and_nonfinite_points)
{
  point_registration_spec spec; spec.target_frame = "robot";
  point_registration_query query(spec);
  auto frame = std::make_shared<const point_frame>();
  const float xyz[]{0, 0, 0, NAN, 0, 0, INFINITY, 0, 0, 4, 0, 0};
  EXPECT_THROW(query.read(nullptr, 4, {0}, xyz), std::invalid_argument);
  EXPECT_THROW(query.read(frame, 4, {0}, nullptr), std::invalid_argument);
  EXPECT_THROW(query.read(frame, 4, {4}, xyz), std::invalid_argument);
  const auto result = query.read(frame, 4, {0, 1, 2, 3}, xyz);
  EXPECT_EQ(result->points.size(), 1U);
  EXPECT_EQ(query.num_registered_points(), 4U);
  EXPECT_THROW(query.read(frame, 5, {0}, xyz), std::invalid_argument);
  EXPECT_TRUE(query.read(frame, 4, {}, nullptr)->points.empty());
  EXPECT_EQ(query.num_registered_points(), 4U);
  EXPECT_EQ(query.read(frame, 4, {0, 1, 2, 3}, xyz)->points, result->points);
}

TEST(point_cloud_store, registration_acquisition_pose_is_per_frame_without_new_query)
{
  point_registration_spec spec; spec.target_frame = "robot";
  point_registration_query query(spec);
  auto frame = std::make_shared<const point_frame>();
  const float xyz[]{.1f, 0, 0};
  const auto first = query.read(frame, 1, {0}, xyz);
  std::array<float, 7> moved_pose{{.5f, 0, 0, 0, 0, 0, 1}};
  EXPECT_THROW(query.read(frame, 1, {0}, xyz, moved_pose), std::invalid_argument);
  frame = std::make_shared<const point_frame>();
  const float moved_xyz[]{.6f, 0, 0};
  const auto moved = query.read(frame, 1, {0}, moved_xyz, moved_pose);
  EXPECT_NE(first->points, moved->points);
  EXPECT_EQ(query.num_registered_points(), 1U);
  moved_pose[0] = NAN;
  EXPECT_THROW(query.read(frame, 1, {0}, moved_xyz, moved_pose), std::invalid_argument);
}

TEST(point_cloud_store, registration_concurrent_consumers_share_result)
{
  point_registration_spec spec; spec.target_frame = "robot";
  point_registration_query query(spec);
  const auto frame = std::make_shared<const point_frame>();
  const float xyz[]{.1f, 0, 0, .2f, 0, 0};
  std::array<std::shared_ptr<const point_registration>, 4> results;
  std::vector<std::thread> threads;
  for (std::size_t idx = 0; idx < results.size(); ++idx) {
    threads.emplace_back([&, idx] {results[idx] = query.read(frame, 2, {1, 0}, xyz);});
  }
  for (auto &thread : threads) {thread.join();}
  for (const auto &result : results) {EXPECT_EQ(result, results.front());}
  EXPECT_EQ(query.num_registered_points(), 2U);
}

TEST(point_cloud_store, bucket_query_preserves_original_indices_without_xyz_reload)
{
  world_point_bucket_index point_idx(.2);
  point_idx.begin_frame(5);
  point_idx.add_point({.01f, 0, 0}, 4);
  point_idx.add_point({NAN, 0, 0}, 1);
  point_idx.add_point({-.05f, 0, 0}, 2);
  point_idx.add_point({5, 0, 0}, 0);
  std::map<std::uint32_t, float> points;
  const auto stats = point_idx.query_aabb_with_source(Eigen::Vector3d::Constant(-.1),
    Eigen::Vector3d::Constant(.1), [&](const auto &point, std::uint32_t source_idx) {
      points.emplace(source_idx, point.x());
    });
  EXPECT_EQ(stats.accepted_point_num, 2U);
  ASSERT_EQ(points.size(), 2U);
  EXPECT_FLOAT_EQ(points.at(4), .01f);
  EXPECT_FLOAT_EQ(points.at(2), -.05f);
  std::size_t num_points = 0;
  point_idx.visit_points_with_source([&](const auto &, std::uint32_t source_idx) {
    EXPECT_NE(source_idx, 1U);
    ++num_points;
  });
  EXPECT_EQ(num_points, 3U);
  point_idx.begin_frame(1);
  point_idx.add_point({.02f, 0, 0}, 0);
  points.clear();
  point_idx.query_aabb_with_source(Eigen::Vector3d::Constant(-1),
    Eigen::Vector3d::Constant(1), [&](const auto &point, std::uint32_t source_idx) {
      points.emplace(source_idx, point.x());
    });
  ASSERT_EQ(points.size(), 1U);
  EXPECT_FLOAT_EQ(points.at(0), .02f);
}

TEST(point_cloud_store, roi_only_frame_shares_source_without_world_copy)
{
  auto channel = shared_point_frames("test_roi_only");
  channel->claim_writer(channel.get());
  point_frame frame;
  frame.source_owner = std::make_shared<int>(42);
  frame.roi_points = std::make_shared<roi_point_membership>();
  frame.frame_id = "robot";
  channel->publish(channel.get(), frame);
  EXPECT_EQ(channel->latest()->source_owner, frame.source_owner);
  EXPECT_EQ(channel->latest()->roi_points, frame.roi_points);
  EXPECT_FALSE(channel->latest()->point_idx);
  frame.source_owner.reset();
  EXPECT_THROW(channel->publish(channel.get(), frame), std::logic_error);
  channel->release_writer(channel.get());
}

TEST(point_cloud_store, shared_identity_and_writer_exclusion)
{
  auto writer = shared_point_frames("test_identity");
  auto reader = shared_point_frames("test_identity");
  ASSERT_EQ(writer.get(), reader.get());
  int first, second;
  writer->claim_writer(&first);
  EXPECT_THROW(reader->claim_writer(&second), std::logic_error);
  auto point_idx = std::make_shared<world_point_bucket_index>(.2);
  point_idx->begin_frame(2);
  point_idx->add_point({-.25f, 0, 0});
  point_idx->add_point({.35f, 0, 0});
  auto payload = std::make_shared<int>(73);
  point_frame frame;
  frame.point_idx = point_idx; frame.source_owner = payload; frame.frame_id = "world";
  writer->publish(&first, frame);
  const auto snapshot = reader->latest();
  ASSERT_TRUE(snapshot);
  EXPECT_EQ(snapshot->point_idx.get(), point_idx.get());
  EXPECT_EQ(snapshot->source_owner.get(), payload.get());
  EXPECT_EQ(snapshot->revision, 1U);
  EXPECT_THROW(writer->publish(&second, frame), std::logic_error);
  writer->release_writer(&second);
  EXPECT_EQ(reader->latest(), snapshot);
  writer->release_writer(&first);
  EXPECT_FALSE(reader->latest());
  EXPECT_EQ(snapshot->point_idx->point_num(), 2U);
  reader->claim_writer(&second);
  reader->publish(&second, frame);
  EXPECT_EQ(reader->latest()->revision, 2U);
  reader->release_writer(&second);
}

TEST(point_cloud_store, snapshot_lifetime_and_reuse)
{
  auto channel = shared_point_frames("test_lifetime");
  channel->claim_writer(channel.get());
  auto point_idx = std::make_shared<world_point_bucket_index>(.2);
  point_idx->begin_frame(1); point_idx->add_point({1, 2, 3});
  point_frame frame; frame.point_idx = point_idx; frame.frame_id = "world";
  channel->publish(channel.get(), std::move(frame));
  auto old = channel->latest();
  EXPECT_FALSE(point_idx.unique());
  auto replacement = std::make_shared<world_point_bucket_index>(.2);
  replacement->begin_frame(0);
  point_frame empty; empty.point_idx = replacement; empty.frame_id = "world";
  channel->publish(channel.get(), std::move(empty));
  EXPECT_EQ(channel->latest()->point_idx->point_num(), 0U);
  EXPECT_EQ(old->point_idx->point_num(), 1U);
  old.reset();
  EXPECT_TRUE(point_idx.unique());
  channel->release_writer(channel.get());
  channel.reset();
  EXPECT_FALSE(shared_point_frames("test_lifetime")->latest());
}

TEST(point_cloud_store, original_bucket_query_and_invalid_inputs)
{
  EXPECT_THROW(shared_point_frames(""), std::invalid_argument);
  EXPECT_THROW(world_point_bucket_index(0), std::invalid_argument);
  world_point_bucket_index point_idx(.2);
  point_idx.begin_frame(4);
  point_idx.add_point({-.11f, 0, 0}); point_idx.add_point({-.09f, 0, 0});
  point_idx.add_point({.09f, 0, 0}); point_idx.add_point({NAN, 0, 0});
  std::size_t num = 0;
  const auto stats = point_idx.query_aabb(Eigen::Vector3d::Constant(-.1),
      Eigen::Vector3d::Constant(.1), [&](const auto &) {++num;});
  EXPECT_EQ(num, 2U); EXPECT_EQ(stats.accepted_point_num, num);
  EXPECT_EQ(point_idx.nonfinite_point_num(), 1U);
  num = 0; point_idx.visit_points([&](const auto &) {++num;});
  EXPECT_EQ(num, 3U);
  point_idx.begin_frame(0); EXPECT_EQ(point_idx.point_num(), 0U);
  num = 0; point_idx.visit_points([&](const auto &) {++num;}); EXPECT_EQ(num, 0U);
  auto channel = shared_point_frames("test_invalid");
  channel->claim_writer(channel.get());
  EXPECT_THROW(channel->publish(channel.get(), {}), std::logic_error);
  channel->release_writer(channel.get());
}

TEST(point_cloud_store, dense_sparse_counts_match_independent_reference)
{
  point_cell_spec spec;
  spec.size = {.1, .2, .15}; spec.origin = {.025, -.04, .05};
  spec.min_corner = Eigen::Vector3d::Constant(-3); spec.max_corner = -spec.min_corner;
  spec.enable_exclusion = true;
  spec.exclude_min = Eigen::Vector3d::Constant(-.25); spec.exclude_max = -spec.exclude_min;
  point_cell_counts dense(spec);
  spec.max_dense_voxel_num = 0;
  point_cell_counts sparse(spec);
  ASSERT_TRUE(dense.has_dense_lookup()); ASSERT_FALSE(sparse.has_dense_lookup());
  std::mt19937 random(260926); std::uniform_real_distribution<float> draw(-4, 4);
  for (int frame = 0; frame < 3; ++frame) {
    dense.begin_frame(); sparse.begin_frame();
    std::map<std::tuple<int, int, int>, uint32_t> expected;
    for (int idx = 0; idx < (frame == 1 ? 0 : 100000); ++idx) {
      const Eigen::Vector3f point(draw(random), draw(random), draw(random));
      dense.add_point(point); sparse.add_point(point);
      if (point.cwiseAbs().maxCoeff() > 3 || point.cwiseAbs().maxCoeff() <= .25) {continue;}
      ++expected[{int(std::floor((double(point.x())-.025)/.1)),
        int(std::floor((double(point.y())+.04)/.2)), int(std::floor((double(point.z())-.05)/.15))}];
    }
    dense.add_point({NAN, 0, 0}); sparse.add_point({INFINITY, 0, 0});
    ASSERT_EQ(dense.cells().size(), expected.size()); ASSERT_EQ(sparse.cells().size(), expected.size());
    for (const auto &entry : expected) {
      const world_bucket_key key{std::get<0>(entry.first), std::get<1>(entry.first), std::get<2>(entry.first)};
      ASSERT_NE(dense.find(key), point_cell_counts::no_cell);
      ASSERT_NE(sparse.find(key), point_cell_counts::no_cell);
      EXPECT_EQ(dense.cells()[dense.find(key)].num_points, entry.second);
      EXPECT_EQ(sparse.cells()[sparse.find(key)].num_points, entry.second);
    }
    EXPECT_EQ(dense.find({99999, 0, 0}), point_cell_counts::no_cell);
  }
}

TEST(point_cloud_store, cell_faces_exclusion_and_validation)
{
  point_cell_spec spec; spec.size = Eigen::Vector3d::Constant(.25);
  point_cell_counts cells(spec);
  for (const float x : {-1.F, -.25F, 0.F, .25F, 1.F}) {cells.add_point({x, 0, 0});}
  ASSERT_EQ(cells.cells().size(), 5U);
  EXPECT_NE(cells.find({-4, 0, 0}), point_cell_counts::no_cell);
  EXPECT_NE(cells.find({4, 0, 0}), point_cell_counts::no_cell);
  spec.enable_exclusion = true; spec.exclude_min = {-.25, -.25, -.25}; spec.exclude_max = -spec.exclude_min;
  point_cell_counts excluded(spec); excluded.add_point({-.25F, 0, 0}); excluded.add_point({.25F, 0, 0});
  EXPECT_TRUE(excluded.cells().empty());
  spec.size.x() = 0; EXPECT_THROW(point_cell_counts{spec}, std::invalid_argument);
  spec.size.x() = NAN; EXPECT_THROW(point_cell_counts{spec}, std::invalid_argument);
  spec.size.x() = .1; spec.origin.z() = INFINITY;
  EXPECT_THROW(point_cell_counts{spec}, std::invalid_argument);
  spec.origin.z() = 0; spec.min_corner.x() = 2;
  EXPECT_THROW(point_cell_counts{spec}, std::invalid_argument);
}

TEST(point_cloud_store, shared_cell_result_is_once_per_snapshot_and_immutable)
{
  auto channel = shared_point_frames("test_cell_cache");
  point_cell_spec spec;
  auto query = channel->cell_query(spec);
  EXPECT_EQ(query, channel->cell_query(spec));
  spec.origin.x() = .025; EXPECT_NE(query, channel->cell_query(spec));
  spec.origin.x() = 0; spec.size.x() = .2; EXPECT_NE(query, channel->cell_query(spec));
  spec.size.x() = .1; spec.enable_exclusion = true; EXPECT_NE(query, channel->cell_query(spec));
  auto idx = std::make_shared<world_point_bucket_index>(.2);
  idx->begin_frame(2); idx->add_point({.01F, 0, 0}); idx->add_point({.02F, 0, 0});
  auto frame = std::make_shared<point_frame>(); frame->point_idx = idx;
  auto first = query->read(frame);
  ASSERT_EQ(first->cells().size(), 1U); EXPECT_EQ(first->cells()[0].num_points, 2U);
  EXPECT_EQ(first, query->read(frame));
  std::shared_ptr<const point_cell_counts> a, b;
  auto concurrent_frame = std::make_shared<point_frame>(*frame);
  std::thread left([&] {a = query->read(concurrent_frame);});
  std::thread right([&] {b = query->read(concurrent_frame);});
  left.join(); right.join(); EXPECT_EQ(a, b);
  EXPECT_EQ(a->cells()[0].num_points, first->cells()[0].num_points);
  auto empty_idx = std::make_shared<world_point_bucket_index>(.2); empty_idx->begin_frame(0);
  auto next = std::make_shared<point_frame>(); next->point_idx = empty_idx;
  // revision未設定でも別snapshotとして識別。旧結果の読取中の上書きなし。
  auto empty = query->read(next);
  EXPECT_TRUE(empty->cells().empty()); EXPECT_EQ(first->cells()[0].num_points, 2U);
  EXPECT_EQ(query->read(frame)->cells()[0].num_points, 2U);
  EXPECT_TRUE(empty->cells().empty());
  EXPECT_THROW(query->read(nullptr), std::invalid_argument);
}

TEST(point_cloud_store, adaptive_query_matches_point_and_bucket_reference)
{
  world_point_bucket_index idx(.25); idx.begin_frame(5000);
  std::vector<Eigen::Vector3f> points;
  for (int x = -20; x <= 20; ++x) {
    for (int y = -20; y <= 20; ++y) {points.push_back({x*.125F, y*.125F, 0}); idx.add_point(points.back());}
  }
  for (const double bound : {.13, 1.1, 3., 1000000.}) {
    std::size_t expected_points = 0, expected_buckets = 0, candidate_points = 0;
    for (const auto &point : points) {if (point.cwiseAbs().maxCoeff() <= bound) {++expected_points;}}
    idx.visit_buckets([&](const world_bucket_key &key, std::size_t num) {
      const int lo = static_cast<int>(std::floor(-bound/.25)), hi = static_cast<int>(std::floor(bound/.25));
      if (key.x >= lo && key.x <= hi && key.y >= lo && key.y <= hi && key.z >= lo && key.z <= hi) {
        ++expected_buckets; candidate_points += num;
      }
    });
    std::size_t actual = 0;
    auto stats = idx.query_aabb(Eigen::Vector3d::Constant(-bound), Eigen::Vector3d::Constant(bound),
      [&](const auto &) {++actual;});
    EXPECT_EQ(actual, expected_points); EXPECT_EQ(stats.accepted_point_num, actual);
    EXPECT_EQ(stats.existing_bucket_num, expected_buckets); EXPECT_EQ(stats.candidate_point_num, candidate_points);
  }
}
