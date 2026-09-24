// 同一プロセス内での木の方式・参照表キャッシュの交互比較。
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <functional>
#include <memory>
#include <optional>
#include <numeric>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>
#include <ais_gng_msgs/msg/topological_map.hpp>
#include <ais_gng_feature_msgs/msg/topological_node_feature_array.hpp>
#include <gng_control_msgs/msg/grasp_candidate_array.hpp>
#include <tf2/LinearMath/Transform.h>

using map_type = ais_gng_msgs::msg::TopologicalMap;
using features_type = ais_gng_feature_msgs::msg::TopologicalNodeFeatureArray;
using source_type = gng_control_msgs::msg::GraspCandidateArray;
using lookup_type = std::function<std::optional<tf2::Transform>(const std::string &, const std::string &)>;
using clock_type = std::chrono::steady_clock;
struct result_type { map_type map; std::vector<int32_t> ids; };
struct runner {
  virtual ~runner() = default;
  virtual void update_map(map_type::ConstSharedPtr map) = 0;
  virtual void update_features(features_type::ConstSharedPtr features) = 0;
  virtual result_type select(const map_type *, const source_type &, const features_type *, const lookup_type &, bool = true) = 0;
};
#include "variants.hpp"

double elapsed_ms(clock_type::time_point start) {
  return std::chrono::duration<double,std::milli>(clock_type::now()-start).count();
}
std::unique_ptr<runner> make_runner(int idx) {
  switch (idx) {
    case 0: return std::make_unique<old_before_runner>();
    case 1: return std::make_unique<bsp_before_runner>();
    case 2: return std::make_unique<old_cached_runner>();
    case 3: return std::make_unique<bsp_cached_runner>();
    case 4: return std::make_unique<bsp_cached_runner>(false,true);
    case 5: return std::make_unique<bsp_cached_runner>(true,false);
    default: return std::make_unique<bsp_incremental_runner>();
  }
}
struct timings {
  std::vector<double> map_update, feature_update, select, total;
};
double median(std::vector<double> values) {
  std::sort(values.begin(),values.end());
  return values[values.size()/2];
}
int main(int argc, char **argv) {
  if (argc != 3) throw std::runtime_error("入力座標と試行番号が必要です");
  const int trial = std::stoi(argv[2]);
  auto initial_map = std::make_shared<map_type>();
  initial_map->header.frame_id = "base";
  auto initial_features = std::make_shared<features_type>();
  std::ifstream input(argv[1]);
  double x,y,z;
  while (input >> x >> y >> z) {
    auto &node = initial_map->nodes.emplace_back();
    node.id = initial_map->nodes.size()-1;
    node.pos.x=x; node.pos.y=y; node.pos.z=z;
    node.normal.z=1;
    node.label=node.id%13 == 0 ? map_type::WALL : 1;
    auto &feature = initial_features->features.emplace_back();
    feature.node_id=node.id;
    feature.manip_valid=true;
    feature.manip_condition_number=1+node.id%100;
  }
  if (initial_map->nodes.empty()) throw std::runtime_error("空の入力座標");
  tf2::Quaternion rotation; rotation.setRPY(0.3,-0.2,0.7);
  const tf2::Transform transform(rotation,tf2::Vector3(0.31,-0.17,0.23));
  const lookup_type lookup=[&](const auto &,const auto &){return transform;};
  const std::vector<std::string> names{"old_before","bsp_before","old_cached","bsp_cached","bsp_features","bsp_output","bsp_incremental"};
  const int num_methods = names.size();
  std::mt19937 random(20260924+trial);
  std::size_t num_checks=0;
  std::cout << std::setprecision(9);
  for (const std::string scenario : {"stable","features","metadata","moving","occasional","sparse","publish_30hz"}) {
    for (const int num_candidates : {1,20,100}) {
      source_type source;
      source.header.frame_id="base"; source.evaluation_header.frame_id="reach";
      source.voxel_size=0.05;
      for (int idx=0;idx<num_candidates;++idx) {
        const auto &pos=initial_map->nodes[(idx*103)%initial_map->nodes.size()].pos;
        auto &candidate=source.candidates.emplace_back();
        candidate.state=gng_control_msgs::msg::GraspCandidate::INSIDE;
        candidate.pose.position.x=pos.x; candidate.pose.position.y=pos.y; candidate.pose.position.z=pos.z;
        candidate.pose.orientation.w=1;
      }
      std::vector<std::unique_ptr<runner>> methods;
      std::vector<timings> measured(num_methods);
      for (int idx=0;idx<num_methods;++idx) methods.push_back(make_runner(idx));
      auto map=initial_map;
      auto features=initial_features;
      for (auto &method:methods) { method->update_map(map); method->update_features(features); }
      // 初期構築の測定。オブジェクト確保・参照表の初期化も対象。
      for (int iter=0;iter<12;++iter) {
        std::vector<int> order(num_methods); std::iota(order.begin(),order.end(),0); std::shuffle(order.begin(),order.end(),random);
        for (int idx:order) {
          const auto start=clock_type::now();
          auto method=make_runner(idx);
          method->update_map(map); method->update_features(features);
          const double ms=elapsed_ms(start);
          if (iter>=2) std::cout << "initial scenario=" << scenario << " candidates=" << num_candidates
              << " method=" << names[idx] << " ms=" << ms << '\n';
        }
      }
      for (int iter=0;iter<70;++iter) {
        // 旧メッセージ破棄を最後の方式だけへ計上しない共通の寿命保持。
        const auto previous_map = map;
        const auto previous_features = features;
        const bool has_map_update=scenario=="metadata" || scenario=="moving" || scenario=="sparse" || (scenario=="occasional" && iter%20==0);
        const bool has_feature_update=scenario=="features";
        if (has_map_update) {
          map=std::make_shared<map_type>(*initial_map);
          map->frame_number=iter;
          // 同じ座標の再配信と座標変更の区別。ID変更も出力参照表へ反映。
          if (scenario!="metadata") for (auto &node:map->nodes)
            if (scenario!="sparse" || node.id%20==0) node.pos.x+=0.001F*(iter%7+1);
          map->nodes[0].id=60000+iter;
        }
        if (has_feature_update) {
          features=std::make_shared<features_type>(*initial_features);
          for (auto &feature:features->features) feature.manip_condition_number=1+(feature.node_id+iter)%100;
        }
        std::vector<map_type::ConstSharedPtr> map_updates;
        std::vector<features_type::ConstSharedPtr> feature_updates;
        if (scenario=="publish_30hz") {
          // 配信30Hz・選択5Hz相当。1選択までに同一座標の6回受信。
          for (int batch=0;batch<6;++batch) {
            auto next_map=std::make_shared<map_type>(*initial_map);
            next_map->frame_number=iter*6+batch;
            map_updates.push_back(next_map);
            feature_updates.push_back(std::make_shared<features_type>(*initial_features));
          }
          map=std::const_pointer_cast<map_type>(map_updates.back());
          features=std::const_pointer_cast<features_type>(feature_updates.back());
        }
        const auto expected=methods[0]->select(map.get(),source,features.get(),lookup,false);
        std::vector<int> order(num_methods); std::iota(order.begin(),order.end(),0); std::shuffle(order.begin(),order.end(),random);
        for (int idx:order) {
          auto &method=*methods[idx];
          const auto begin=clock_type::now();
          for (const auto &next:map_updates) method.update_map(next);
          if (has_map_update) method.update_map(map);
          const double map_ms=elapsed_ms(begin);
          const auto feature_begin=clock_type::now();
          for (const auto &next:feature_updates) method.update_features(next);
          if (has_feature_update) method.update_features(features);
          const double feature_ms=elapsed_ms(feature_begin);
          const auto select_begin=clock_type::now();
          auto actual=method.select(map.get(),source,features.get(),lookup);
          const double select_ms=elapsed_ms(select_begin);
          const double total_ms=elapsed_ms(begin);
          if (actual.ids!=expected.ids || actual.map!=expected.map)
            throw std::runtime_error("出力不一致: "+names[idx]+" "+scenario+" "+std::to_string(iter));
          ++num_checks;
          if (iter>=10) {
            auto &values=measured[idx];
            values.map_update.push_back(map_ms); values.feature_update.push_back(feature_ms);
            values.select.push_back(select_ms); values.total.push_back(total_ms);
          }
        }
      }
      for (int idx=0;idx<num_methods;++idx) {
        const auto &values=measured[idx];
        const auto mean=[](const auto &v){double sum=0;for(double x:v)sum+=x;return sum/v.size();};
        std::cout << "result scenario=" << scenario << " candidates=" << num_candidates << " method=" << names[idx]
            << " nodes=" << initial_map->nodes.size() << " samples=60"
            << " map_ms=" << mean(values.map_update) << " features_ms=" << mean(values.feature_update)
            << " select_ms=" << mean(values.select) << " total_ms=" << mean(values.total)
            << " median_select_ms=" << median(values.select) << " median_total_ms=" << median(values.total) << '\n';
      }
      std::cout << "case_done checks=" << num_checks << " mismatches=0" << std::endl;
    }
  }
}
