#include "ais_gng/boundary_attention.hpp"
#include "selection_reference.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdexcept>

// 実入力でのkd-tree構築・近傍検索・重点重み混合の有限回数測定。
int main(int argc, char **argv) {
    if (argc < 2 || argc > 4 || (argc == 4 && std::string(argv[3]) != "reference")) {return 2;}
    const bool is_reference = argc == 4;
    std::ifstream input(argv[1], std::ios::binary);
    uint32_t num_anchors = 0, num_points = 0;
    input.read(reinterpret_cast<char *>(&num_anchors), sizeof(num_anchors));
    input.read(reinterpret_cast<char *>(&num_points), sizeof(num_points));
    if (!input || num_anchors > 65534 || num_points > 1000000) {return 2;}
    std::vector<fuzzrobo::boundary_attention::point> anchors(num_anchors);
    std::vector<float> points(static_cast<std::size_t>(num_points) * 3);
    input.read(reinterpret_cast<char *>(anchors.data()), anchors.size() * sizeof(anchors[0]));
    input.read(reinterpret_cast<char *>(points.data()), points.size() * sizeof(float));
    if (!input) {return 2;}
    std::vector<double> samples;
    std::size_t num_selected = 0;
    for (int iter = 0; iter < 9; ++iter) {
        const auto start = std::chrono::steady_clock::now();
        const auto weights = is_reference ? selection_reference::make_weights(points.data(), num_points, anchors, .3) :
            fuzzrobo::boundary_attention::make_weights(points.data(), num_points, anchors, .3);
        const auto priority = fuzzrobo::boundary_attention::mix({}, 0, {}, 0, weights, .5);
        const auto elapsed = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count();
        num_selected = priority.ids.size();
        if (iter >= 2) {samples.push_back(elapsed);}
        if (iter == 8 && argc >= 3) {
            // 最適化前後の候補重み・添字・混合重みの完全一致用出力。
            std::ofstream output(argv[2], std::ios::binary);
            output.write(reinterpret_cast<const char *>(weights.data()), weights.size() * sizeof(float));
            output.write(reinterpret_cast<const char *>(priority.ids.data()), priority.ids.size() * sizeof(uint32_t));
            output.write(reinterpret_cast<const char *>(priority.weights.data()), priority.weights.size() * sizeof(float));
            if (!output) {return 2;}
        }
    }
    std::sort(samples.begin(), samples.end());
    std::cout << "{\"is_reference\":" << (is_reference ? "true" : "false")
              << ",\"num_points\":" << num_points << ",\"num_anchors\":" << num_anchors
              << ",\"num_selected\":" << num_selected << ",\"min_ms\":" << samples.front()
              << ",\"median_ms\":" << samples[samples.size() / 2]
              << ",\"max_ms\":" << samples.back() << "}\n";
}
