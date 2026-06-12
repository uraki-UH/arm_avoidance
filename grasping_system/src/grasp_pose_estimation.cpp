// SPDX-License-Identifier: Apache-2.0

#include <core/grasp_candidate.hpp>

#include <optional>
#include <string>
#include <vector>

namespace grasping_system::pose_estimation
{

// 物体に対する把持手先姿勢候補を集める役。
// ここでは探索・生成の「枠組み」だけを置き、評価ロジックは後で実装する。
class GraspPoseCandidateCollector
{
public:
  using Candidate = core::GraspCandidate;

  std::vector<Candidate> collect(const std::string &object_id) const
  {
    (void)object_id;
    return {};
  }
};

// 候補群の中から採用する把持姿勢を決める役。
// ここでは選択フローの枠だけを置き、スコアリングや制約判定は後で実装する。
class GraspPoseSelector
{
public:
  using Candidate = core::GraspCandidate;

  std::optional<Candidate> select(const std::vector<Candidate> &candidates) const
  {
    if (candidates.empty()) {
      return std::nullopt;
    }
    return candidates.front();
  }
};

// 使い方の土台になる窓口関数。
// 実運用のロジックは別途追加していく前提で、現時点では候補取得と選択の流れだけを表現する。
std::optional<core::GraspCandidate> estimateGraspPose(const std::string &object_id)
{
  GraspPoseCandidateCollector collector;
  GraspPoseSelector selector;

  const auto candidates = collector.collect(object_id);
  return selector.select(candidates);
}

}  // namespace grasping_system::pose_estimation
