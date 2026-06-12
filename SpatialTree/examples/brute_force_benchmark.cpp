#include <iostream>
#include <vector>
#include <chrono>
#include <random>
#include <cmath>
#include <SpatialTree/GNG/GNG.hpp>

using namespace SpatialTree;

template<int Dim>
void run_linear_search_benchmark(int num_nodes, int query_count) {
    std::mt19937 gen(42);
    std::uniform_real_distribution<double> dist(0.0, 100.0);

    // 1. データ点（10万個）を用意
    std::vector<Point<double, Dim>> data(num_nodes);
    for (int i = 0; i < num_nodes; ++i) {
        for (int d = 0; d < Dim; ++d) {
            data[i][d] = dist(gen);
        }
    }

    // 2. クエリ点（1000個）を用意
    std::vector<Point<double, Dim>> queries(query_count);
    for (int i = 0; i < query_count; ++i) {
        for (int d = 0; d < Dim; ++d) {
            queries[i][d] = dist(gen);
        }
    }

    // 3. 総当たり（O(N)）で最近傍・第2近傍をさがす時間の計測
    auto start_time = std::chrono::high_resolution_clock::now();

    double dummy_sum = 0.0; // 実行時最適化によるループ削除を防ぐため

    for (int q = 0; q < query_count; ++q) {
        const auto& target = queries[q];
        
        double min_dist1 = std::numeric_limits<double>::max();
        double min_dist2 = std::numeric_limits<double>::max();

        for (int i = 0; i < num_nodes; ++i) {
            double sq_dist = (data[i] - target).squaredNorm();
            min_dist2 = std::min(min_dist2, std::max(min_dist1, sq_dist));
            min_dist1 = std::min(min_dist1, sq_dist);
        }
        dummy_sum += min_dist1 + min_dist2; // 最適化防止
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    std::cout << Dim << "D Linear Search (N=" << num_nodes << ") | "
              << "Average Time per query: " << (double)duration.count() / query_count << " us "
              << "(dummy=" << dummy_sum << ")" << std::endl;
}

int main() {
    const int N = 100000;
    const int queries = 1000;

    std::cout << "--- Brute Force Nearest Neighbor Benchmark (Finding Best 2) ---" << std::endl;
    std::cout << "Data Points: " << N << ", Query Points: " << queries << "\n" << std::endl;

    run_linear_search_benchmark<2>(N, queries);
    run_linear_search_benchmark<3>(N, queries);
    run_linear_search_benchmark<5>(N, queries);
    run_linear_search_benchmark<7>(N, queries);

    std::cout << "\nBenchmark Complete." << std::endl;

    return 0;
}
