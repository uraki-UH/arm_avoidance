#include <fuzzrobo/libgng/api.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <random>
#include <string_view>
#include <vector>

namespace {

constexpr uint32_t kNodeNum = 2048;
constexpr uint32_t kEventsPerFrame = 5000;
constexpr uint32_t kFrameNum = 5000;

struct CovarianceStats {
    double count = 0.0;
    std::array<double, 3> mean{0.0, 0.0, 0.0};
    std::array<double, 9> m2{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

void updateCovariance(CovarianceStats &stats, const GngTrainingEvent &event) {
    const std::array<double, 3> point{
        static_cast<double>(event.residual.x),
        static_cast<double>(event.residual.y),
        static_cast<double>(event.residual.z),
    };
    stats.count += 1.0;
    const double n = stats.count;
    const std::array<double, 3> delta{
        point[0] - stats.mean[0],
        point[1] - stats.mean[1],
        point[2] - stats.mean[2],
    };
    stats.mean[0] += delta[0] / n;
    stats.mean[1] += delta[1] / n;
    stats.mean[2] += delta[2] / n;
    const std::array<double, 3> delta2{
        point[0] - stats.mean[0],
        point[1] - stats.mean[1],
        point[2] - stats.mean[2],
    };
    stats.m2[0] += delta[0] * delta2[0];
    stats.m2[1] += delta[0] * delta2[1];
    stats.m2[2] += delta[0] * delta2[2];
    stats.m2[3] += delta[1] * delta2[0];
    stats.m2[4] += delta[1] * delta2[1];
    stats.m2[5] += delta[1] * delta2[2];
    stats.m2[6] += delta[2] * delta2[0];
    stats.m2[7] += delta[2] * delta2[1];
    stats.m2[8] += delta[2] * delta2[2];
}

using TrainingEventCallback = void (*)(CovarianceStats *, const GngTrainingEvent &);

[[gnu::noinline]] void updateCovarianceCallback(
    CovarianceStats *stats,
    const GngTrainingEvent &event) {
    updateCovariance(stats[event.winner_node_id], event);
}

double calculateChecksum(const std::vector<CovarianceStats> &stats) {
    double sum = 0.0;
    for (const auto &node : stats) {
        sum += node.count + node.mean[0] + node.m2[0];
    }
    return sum;
}

template <typename Function>
double measureMs(Function &&function) {
    const auto begin = std::chrono::steady_clock::now();
    function();
    const auto end = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

void printResult(std::string_view name, double total_ms, double checksum) {
    const double event_num = static_cast<double>(kEventsPerFrame) * kFrameNum;
    std::cout << std::fixed << std::setprecision(3)
              << name
              << " total_ms=" << total_ms
              << " ns_per_event=" << total_ms * 1000000.0 / event_num
              << " ms_per_5000_events=" << total_ms / kFrameNum
              << " checksum=" << checksum << '\n';
}

}  // namespace

int main() {
    static_assert(sizeof(GngTrainingEvent) == 20, "GngTrainingEventのABIサイズ");

    std::mt19937 generator(42);
    std::uniform_int_distribution<uint32_t> node_distribution(0, kNodeNum - 1);
    std::normal_distribution<float> residual_distribution(0.0f, 0.04f);
    std::vector<GngTrainingEvent> source_events(kEventsPerFrame);
    for (auto &event : source_events) {
        event.winner_node_id = static_cast<uint16_t>(node_distribution(generator));
        event.winner_rank = 1;
        event.winner_node_frame = 1;
        event.residual = {
            residual_distribution(generator),
            residual_distribution(generator),
            residual_distribution(generator),
        };
    }

    std::vector<CovarianceStats> direct_stats(kNodeNum);
    const double direct_ms = measureMs([&] {
        for (uint32_t frame_idx = 0; frame_idx < kFrameNum; ++frame_idx) {
            for (const auto &event : source_events) {
                updateCovariance(direct_stats[event.winner_node_id], event);
            }
        }
    });
    printResult("direct", direct_ms, calculateChecksum(direct_stats));

    std::vector<CovarianceStats> callback_stats(kNodeNum);
    TrainingEventCallback callback = updateCovarianceCallback;
    const double callback_ms = measureMs([&] {
        for (uint32_t frame_idx = 0; frame_idx < kFrameNum; ++frame_idx) {
            for (const auto &event : source_events) {
                callback(callback_stats.data(), event);
            }
        }
    });
    printResult("callback", callback_ms, calculateChecksum(callback_stats));

    std::vector<GngTrainingEvent> event_buffer(kEventsPerFrame);
    std::vector<CovarianceStats> buffer_stats(kNodeNum);
    double producer_ms = 0.0;
    double consumer_ms = 0.0;
    const double buffer_total_ms = measureMs([&] {
        for (uint32_t frame_idx = 0; frame_idx < kFrameNum; ++frame_idx) {
            producer_ms += measureMs([&] {
                for (uint32_t event_idx = 0; event_idx < kEventsPerFrame; ++event_idx) {
                    event_buffer[event_idx] = source_events[event_idx];
                }
            });
            consumer_ms += measureMs([&] {
                for (const auto &event : event_buffer) {
                    updateCovariance(buffer_stats[event.winner_node_id], event);
                }
            });
        }
    });
    printResult("buffer_total", buffer_total_ms, calculateChecksum(buffer_stats));
    printResult("buffer_producer", producer_ms, 0.0);
    printResult("buffer_consumer", consumer_ms, 0.0);
    return 0;
}
