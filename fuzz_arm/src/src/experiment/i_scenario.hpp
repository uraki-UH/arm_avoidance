#pragma once
#include <Eigen/Dense>
#include <string>

namespace robot_sim {
namespace experiment {

/**
 * @brief 実験用シナリオインターフェース
 */
class IScenario {
public:
  virtual ~IScenario() = default;

  /**
   * @brief シナリオの初期化
   */
  virtual void reset() = 0;

  /**
   * @brief 現在の目標デカルト座標の取得
   */
  virtual Eigen::Vector3d getCurrentTargetPos() const = 0;

  /**
   * @return 全てのタスクが完了した場合はtrue
   */
  virtual bool onTargetReached() = 0;

  /**
   * @brief 現在の目標に対する到達判定しきい値の取得
   */
  virtual double getTargetTolerance() const = 0;

  /**
   * @brief シナリオの状態更新 (障害物の移動制御など)
   * @param dt 経過時間
   */
  virtual void update(double dt) = 0;

  /**
   * @brief シナリオの名称取得
   */
  virtual std::string getName() const = 0;

  /**
   * @brief 進捗状況の文字列取得 (デバッグ表示用)
   */
  virtual std::string getProgressString() const = 0;
};

} // namespace experiment
} // namespace robot_sim
