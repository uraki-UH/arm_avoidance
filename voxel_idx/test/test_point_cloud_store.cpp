#include <point_cloud_store.hpp>
#include <gtest/gtest.h>
#include <map>
#include <random>
#include <thread>
#include <tuple>

using namespace voxel_idx;

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
