#include <ais_gng/plugin/cluster_classification.hpp>
#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

namespace {
using map_message = ais_gng_msgs::msg::TopologicalMap;

std::unique_ptr<map_message> make_candidates() {
    auto map = std::make_unique<map_message>();
    map->frame_number = 4000;
    map->nodes.resize(64);
    map->clusters.resize(2);
    for (uint32_t idx = 0; idx < 64; ++idx) {
        auto &node = map->nodes[idx];
        node.id = static_cast<uint16_t>(100 + idx);
        const float angle = static_cast<float>(idx % 8) * 0.785398163f;
        node.pos.x = 2.0f + 0.2f * std::cos(angle);
        node.pos.y = 0.2f * std::sin(angle);
        node.pos.z = 0.2f + 0.35f * static_cast<float>((idx % 32) / 8);
        node.normal.x = std::cos(angle);
        node.normal.y = std::sin(angle);
        map->clusters[idx / 32].nodes.push_back(static_cast<uint16_t>(idx));
    }
    for (uint32_t idx = 0; idx < 2; ++idx) {
        auto &cluster = map->clusters[idx];
        cluster.id = 40 + idx;
        cluster.frame = 3997 + idx;
        cluster.label = map_message::UNKNOWN_OBJECT;
        cluster.pos.z = 0.8f;
        cluster.scale.z = 1.2f;
    }
    return map;
}

void expect_skipped(ClusterClassification &classifier, std::unique_ptr<map_message> &map) {
    std::vector<uint32_t> ids, ages;
    std::vector<uint8_t> labels;
    EXPECT_NO_THROW(classifier.classify(map, ids, ages, labels));
    EXPECT_TRUE(ids.empty());
    EXPECT_TRUE(ages.empty());
    EXPECT_TRUE(labels.empty());
}
}

TEST(cluster_classification, empty_and_filtered_candidates) {
    ClusterClassification classifier;
    auto map = std::make_unique<map_message>();
    expect_skipped(classifier, map);
    map = make_candidates();
    for (auto &cluster : map->clusters) {cluster.nodes.resize(29);}
    expect_skipped(classifier, map);
    map = make_candidates();
    for (auto &cluster : map->clusters) {cluster.pos.z = 3.0f;}
    expect_skipped(classifier, map);
}

TEST(cluster_classification, disabled_models) {
    ClusterClassification classifier;
    classifier.setParameter("classify.human", 0, 0.0);
    classifier.setParameter("classify.car", 0, 0.0);
    auto map = make_candidates();
    expect_skipped(classifier, map);
}

TEST(cluster_classification, real_models_receive_members_and_return_feedback) {
    auto context = std::make_shared<rclcpp::Context>();
    context->init(0, nullptr);
    auto node = std::make_shared<rclcpp::Node>("cluster_classification_test",
        rclcpp::NodeOptions().context(context).start_parameter_services(false)
            .start_parameter_event_publisher(false).enable_rosout(false));
    // 人のみ・車のみ・両方の各推論。認識精度ではなく所属添字からの入力経路の確認。
    for (const int mode : {1, 2, 3}) {
        ClusterClassification classifier;
        classifier.setParameter("classify.human", 0, static_cast<double>((mode & 1) != 0));
        classifier.setParameter("classify.car", 0, static_cast<double>((mode & 2) != 0));
        classifier.setParameter("classify.device", 0, std::string("cpu"));
        classifier.init(node.get());
        auto map = make_candidates();
        std::vector<uint32_t> ids, ages;
        std::vector<uint8_t> labels;
        EXPECT_NO_THROW(classifier.classify(map, ids, ages, labels));
        EXPECT_EQ(ids, (std::vector<uint32_t>{40, 41}));
        EXPECT_EQ(ages, (std::vector<uint32_t>{3, 2}));
        EXPECT_EQ(labels.size(), 2U);
        for (const auto &cluster : map->clusters) {
            EXPECT_TRUE(cluster.label_inferred == map_message::UNKNOWN_OBJECT ||
                cluster.label_inferred == map_message::HUMAN ||
                cluster.label_inferred == map_message::CAR);
            EXPECT_TRUE(std::isfinite(cluster.label_reliability));
        }
    }
    node.reset();
    context->shutdown("分類器テスト終了");
}
