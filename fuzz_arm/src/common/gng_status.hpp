// FeatureGNG.hpp
#pragma once
#include <Eigen/Core>
#include <vector>
#include <queue>
#include <mutex>
#include <thread>
#include <functional>
#include <unordered_map>
#include <string>

// センササンプル（最小構成）
struct SensorSample {
    std::vector<double> joint_angles;    // joint space
    Eigen::Vector3f ee_pos;              // end-effector position
    double timestamp = 0.0;
    int object_id = -1;                  // optional anchor id
};

// 抽出器インターフェイス：Sample -> 主ベクトル + メタ特徴量
struct FeatureBundle {
    Eigen::VectorXf primary; // 主表現（GNG の weight と同次元）
    std::unordered_map<std::string, float> meta; // 任意の特徴量（スカラー）
};

struct IFeatureExtractor {
    virtual ~IFeatureExtractor() = default;
    virtual FeatureBundle extract(const SensorSample& s) const = 0;
    // 推奨: returns preferred scale / normalization info
    virtual int outputDim() const = 0;
};

// 距離戦略：primary vector と meta を結合して距離を返す
using DistanceFunc = std::function<float(const Eigen::VectorXf&, const Eigen::VectorXf&,
                                         const std::unordered_map<std::string,float>&,
                                         const std::unordered_map<std::string,float>&)>;

// GNG ノードメタ情報拡張
template <typename VecT>
struct GNGNode {
    int id = -1;
    VecT weight; // primary weight
    std::unordered_map<std::string,float> meta; // meta features (optional)
    float error = 0.0f;
    int anchor_object = -1; // optional
    float confidence = 1.0f;
};

// シンプル汎用GNG（主要操作だけ）
template <typename VecT>
class GenericGNG {
public:
    using NodeT = GNGNode<VecT>;
    GenericGNG(int dim_primary, DistanceFunc dist,
               std::shared_ptr<IFeatureExtractor> extractor = nullptr);
    ~GenericGNG();

    // 非同期学習キュー API（スレッドセーフ）
    void pushSample(const SensorSample& s);
    void startWorker(); // スレッド開始
    void stopWorker();  // 停止

    // シンクロナス学習 (単ステップ)
    void trainOne(const FeatureBundle& f);

    // コールバック（ノード追加等）
    std::function<void(const NodeT&)> onNodeAdded;

private:
    void workerLoop();
    std::mutex mtx_;
    std::queue<SensorSample> queue_;
    std::atomic<bool> running_{false};
    std::thread worker_;

    // GNG 内部データ（簡潔化）
    std::vector<NodeT> nodes_;
    std::vector<std::vector<int>> adjacency_;
    int max_nodes_ = 200;
    int dim_primary_;
    DistanceFunc distance_;
    std::shared_ptr<IFeatureExtractor> extractor_;

    // パラメータ（適宜公開）
    float learnRate1 = 0.08f;
    float learnRate2 = 0.008f;
    int lambda = 100;
    int age_max = 100;

    int step_counter = 0;

    // 内部ヘルパ
    int findClosest(const VecT& v, const std::unordered_map<std::string,float>& meta,
                    int& out_second) const;
    void insertNodeBetween(int q_id, int f_id);
};
