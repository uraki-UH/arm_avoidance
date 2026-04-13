#pragma once

namespace GNG {

struct GngParameters {
  int lambda = 200;
  int max_node_num = 10000;
  int num_samples = 500000;
  int max_iterations = 100000;
  int refine_iterations = 50000;
  int coord_edge_iterations = 100000;
  float learn_rate_s1 = 0.08f;
  float learn_rate_s2 = 0.008f;
  float beta = 0.005f;
  float alpha = 0.5f;
  int max_edge_age = 50;
  int start_node_num = 2;

  // Additional parameters for specific implementations (e.g. offline/AiS)
  int n_best_candidates = 2;
  float ais_threshold = 0.5f;
  int runtime_per_frame = 100;
  float lpf_alpha = 0.01f; // α value for EMA/Cascaded EMA
};

} // namespace GNG
