#include <SpatialTree/GNG/GNG.hpp>
#include <SpatialTree/GNG/GNGPolicy.hpp>
#include <chrono>
#include <iostream>
#include <random>
#include <vector>

using namespace SpatialTree;

struct SimpleNode {
  Point<double, 3> position;
  void *spatial_handle = nullptr;
  int index_in_cell = -1;
};

template<typename Policy>
void run_benchmark(const std::string& name, const std::vector<Point<double, 3>>& initial_points, const std::vector<Point<double, 3>>& deltas) {
    SpatialTreeParams<double> params;
    params.max_nodes_per_cell = 10;
    
    AdaptiveTree<SimpleNode, double, 3, SpatialTraits<SimpleNode, double, 3>, Policy> tree(Point<double, 3>(100, 100, 100), params);
    
    std::vector<SimpleNode> nodes(initial_points.size());
    for(size_t i=0; i<initial_points.size(); ++i) {
        nodes[i].position = initial_points[i];
        tree.add(&nodes[i]);
    }
    
    auto start = std::chrono::high_resolution_clock::now();
    
    for(size_t i=0; i<deltas.size(); ++i) {
        SimpleNode& node = nodes[i % nodes.size()];
        Point<double, 3> next_pos = node.position + deltas[i];
        
        // 領域外に出ないようにクランプ
        for(int d=0; d<3; ++d) {
            if(next_pos[d] < -50.0) next_pos[d] = -49.99;
            if(next_pos[d] > 50.0) next_pos[d] = 49.99;
        }
        
        tree.updatePosition(&node, next_pos, 0.1);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    
    std::cout << name << ": " << duration.count() / 1000.0 << " ms (" 
              << (double)duration.count() / deltas.size() << " us/step)" << std::endl;
}

struct NoHysteresisUpward {
    static constexpr bool enabled = false;
    static constexpr bool use_upward_traversal = true;
};

int main() {
    const int N = 100000;
    const int M = 1000000;
    
    std::mt19937 gen(42);
    std::uniform_real_distribution<double> dist(-50, 50);
    std::uniform_real_distribution<double> move_dist(-0.5, 0.5); // セル境界を跨ぎやすい距離
    
    std::vector<Point<double, 3>> initial_points;
    for(int i=0; i<N; ++i) initial_points.push_back(Point<double, 3>(dist(gen), dist(gen), dist(gen)));
    
    std::vector<Point<double, 3>> deltas;
    for(int i=0; i<M; ++i) deltas.push_back(Point<double, 3>(move_dist(gen), move_dist(gen), move_dist(gen)));
    
    std::cout << "Update Mechanism Benchmark (N=" << N << ", M=" << M << ")" << std::endl;
    std::cout << "------------------------------------------" << std::endl;
    
    // 1. Global Re-insertion (Hysteresis OFF)
    run_benchmark<NoHysteresis>("1. Global Re-insertion (No Hys)", initial_points, deltas);
    
    // 2. Upward Traversal (Hysteresis OFF)
    run_benchmark<NoHysteresisUpward>("2. Upward Traversal (No Hys)", initial_points, deltas);

    // 3. Upward Traversal (Hysteresis ON)
    run_benchmark<AdaptiveHysteresis>("3. Upward Traversal (with Hys)", initial_points, deltas);

    return 0;
}
